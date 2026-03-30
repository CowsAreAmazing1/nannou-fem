use nannou::prelude::*;
use spade::{ConstrainedDelaunayTriangulation, HasPosition, Point2, Triangulation};

use crate::fem::{
    apply_dirichlet_boundary, assemble_poisson_system, build_mesh, conjugate_gradient_solve,
};
use crate::gpu::GpuState;
mod fem;
mod gpu;

// use crate::svg::read_svg;
// mod svg;

fn main() {
    // read_svg("./src/sus.svg");

    nannou::app(model)
        .update(update)
        .loop_mode(LoopMode::refresh_sync())
        .run();
}

struct Vertex {
    pos: Vec2,
}

impl From<Vec2> for Vertex {
    fn from(pos: Vec2) -> Self {
        Self { pos }
    }
}

impl From<Point2<f32>> for Vertex {
    fn from(pos: Point2<f32>) -> Self {
        Self {
            pos: vec2(pos.x, pos.y),
        }
    }
}

impl HasPosition for Vertex {
    type Scalar = f32;

    fn position(&self) -> Point2<Self::Scalar> {
        spade::Point2::new(self.pos.x, self.pos.y)
    }
}

struct Model {
    triangulation: ConstrainedDelaunayTriangulation<Vertex>,
    gpu: GpuState,
    values: Vec<f32>,
}

/// Sets up a constrained Delaunay triangulation, with the given vertices constrained in a closed loop, refines, and returns the triangulation.
fn setup_triangulation(
    to_constrain: &[Vec2],
    half_width: f32,
    half_height: f32,
) -> ConstrainedDelaunayTriangulation<Vertex> {
    let verts = [
        vec2(-half_width, -half_height),
        vec2(half_width, -half_height),
        vec2(half_width, half_height),
        vec2(-half_width, half_height),
    ];

    let mut triangulation: ConstrainedDelaunayTriangulation<Vertex> =
        ConstrainedDelaunayTriangulation::new();

    to_constrain.windows(2).for_each(|window| {
        if let [vert1, vert2] = window {
            triangulation
                .add_constraint_edge(Vertex::from(*vert1), Vertex::from(*vert2))
                .unwrap();
        }
    });

    if let (Some(first), Some(last)) = (to_constrain.first(), to_constrain.last()) {
        triangulation
            .add_constraint_edge(Vertex::from(*last), Vertex::from(*first))
            .unwrap();
    }

    verts.iter().for_each(|&vert| {
        triangulation.insert(Vertex::from(vert)).unwrap();
    });

    // triangulation.refine(spade::RefinementParameters::default());
    triangulation.refine(
        spade::RefinementParameters::default()
            .with_max_additional_vertices(10)
            .with_angle_limit(spade::AngleLimit::from_deg(30.0)),
    );

    triangulation
}

fn normalize_to_unit_interval(values: &[f32]) -> Vec<f32> {
    if values.is_empty() {
        return Vec::new();
    }

    let (min_val, max_val) = values
        .iter()
        .fold((f32::INFINITY, f32::NEG_INFINITY), |(mn, mx), &v| {
            (mn.min(v), mx.max(v))
        });

    let span = max_val - min_val;
    if span.abs() <= 1.0e-12 {
        return vec![0.0; values.len()];
    }

    values
        .iter()
        .map(|&v| ((v - min_val) / span).clamp(0.0, 1.0))
        .collect::<Vec<_>>()
}

fn model(app: &App) -> Model {
    let win = app.new_window().view(view).maximized(true).build().unwrap();
    let rect = app.window(win).unwrap().rect();
    let (_, _, w, h) = rect.x_y_w_h();
    let half_w = w * 0.5;
    let half_h = h * 0.5;

    let radius = 500.0;
    let num = 10;
    // let to_constrain = (0..num)
    //     .map(|i| {
    //         let angle = map_range(i, 0, num, 0.0, TAU);
    //         vec2(radius * angle.cos(), radius * angle.sin())
    //     })
    //     .collect::<Vec<_>>();
    let to_constrain = (0..num)
        .map(|i| {
            let angle = map_range(i, 0, num, 0.0, TAU);
            let rad = radius * (1.0 + 0.25 * (5.0 * angle).sin());
            vec2(rad * angle.cos(), rad * angle.sin())
        })
        .collect::<Vec<_>>();

    let triangulation = setup_triangulation(&to_constrain, half_w, half_h);
    let fem_mesh = build_mesh(&triangulation, &to_constrain);

    let source_radius = radius * 0.18;
    let source_strength = 1.0;
    let mut poisson = assemble_poisson_system(&fem_mesh, |p| {
        if p.length_squared() <= source_radius * source_radius {
            source_strength
        } else {
            0.0
        }
    });
    apply_dirichlet_boundary(&mut poisson, &fem_mesh, |_i, _pos| 0.0);
    let solve = conjugate_gradient_solve(&poisson, 1.0e-4, 5_000);
    let values = normalize_to_unit_interval(&solve.solution);

    let binding = app.main_window();
    let device = binding.device();
    let queue = binding.queue();
    let gpu = GpuState::new(device, queue, &triangulation, binding.rect(), Some(&values));

    println!(
        "Triangulation has {} vertices and {} faces",
        triangulation.vertices().count(),
        triangulation.inner_faces().count()
    );
    println!(
        "FEM step 1: extracted {} nodes, {} elements, {} boundary nodes",
        fem_mesh.positions.len(),
        fem_mesh.elements.len(),
        fem_mesh.is_boundary.iter().filter(|&&b| b).count()
    );
    println!(
        "FEM step 2: assembled Poisson system with {} nodes, {} non-zeros, rhs size {}",
        poisson.node_count(),
        poisson.nnz(),
        poisson.rhs.len()
    );
    println!(
        "FEM step 3: CG converged={}, iterations={}, residual_l2={:.3e}",
        solve.converged, solve.iterations, solve.residual_l2
    );

    Model {
        triangulation,
        gpu,
        values,
    }
}

fn update(app: &App, model: &mut Model, _update: Update) {
    let window = app.main_window();
    let device = window.device();
    let queue = window.queue();
    let rect = window.rect();
    let (_, _, w, h) = rect.x_y_w_h();
    let half_w = w * 0.5;
    let half_h = h * 0.5;

    {
        let radius = 500.0;
        let num = app.elapsed_frames();
        let to_constrain = (0..num)
            .map(|i| {
                let angle = map_range(i, 0, num, 0.0, TAU);
                let rad = radius * (1.0 + 0.25 * (5.0 * angle).sin());
                vec2(rad * angle.cos(), rad * angle.sin())
            })
            .collect::<Vec<_>>();

        model.triangulation = setup_triangulation(&to_constrain, half_w, half_h);

        model.triangulation.refine(
            spade::RefinementParameters::default()
                .with_max_additional_vertices(50 * app.elapsed_frames() as usize)
                .with_max_allowed_area(100.0)
                .with_angle_limit(spade::AngleLimit::from_deg(30.0)),
        );

        let fem_mesh = build_mesh(&model.triangulation, &to_constrain);

        let source_radius = radius * 0.18;
        let source_strength = 1.0;
        let mut poisson = assemble_poisson_system(&fem_mesh, |p| {
            if p.length_squared() <= source_radius * source_radius {
                source_strength
            } else {
                0.0
            }
        });
        apply_dirichlet_boundary(&mut poisson, &fem_mesh, |_i, _pos| 0.0);
        let solve = conjugate_gradient_solve(&poisson, 1.0e-4, 5_000);
        let values = normalize_to_unit_interval(&solve.solution);

        let device = window.device();
        let queue = window.queue();
        let (vertices, tris) = GpuState::prepare_geometry(&model.triangulation, rect);
        model
            .gpu
            .upload_mesh(device, queue, &vertices, &tris, Some(&values));
    }

    // model
    //     .triangulation
    //     .refine(spade::RefinementParameters::default());

    // let (_, _, w, h) = rect.x_y_w_h();
    // let half_w = w * 0.5;
    // let half_h = h * 0.5;

    // let radius = 500.0;
    // let num = 10 * app.elapsed_frames();
    // let to_constrain = (0..num)
    //     .map(|i| {
    //         let angle = map_range(i, 0, num, 0.0, TAU);
    //         vec2(radius * angle.cos(), radius * angle.sin())
    //     })
    //     .collect::<Vec<_>>();

    // model.triangulation = setup_triangulation(to_constrain, half_w, half_h);

    let (vertices, tris) = GpuState::prepare_geometry(&model.triangulation, rect);
    model
        .gpu
        .upload_mesh(device, queue, &vertices, &tris, Some(&model.values));
}

fn view(app: &App, model: &Model, frame: Frame) {
    let window = app.main_window();
    let device = window.device();
    let queue = window.queue();

    let mut encoder = device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
        label: Some("Render Encoder"),
    });

    let render_pass_desc = wgpu::RenderPassDescriptor {
        label: Some("Render Pass"),
        color_attachments: &[Some(wgpu::RenderPassColorAttachment {
            view: frame.texture_view(),
            resolve_target: None,
            ops: wgpu::Operations {
                load: wgpu::LoadOp::Clear(wgpu::Color::BLACK),
                store: true,
            },
        })],
        depth_stencil_attachment: None,
    };

    {
        let mut render_pass = encoder.begin_render_pass(&render_pass_desc);
        render_pass.set_pipeline(&model.gpu.render_pipeline);
        render_pass.set_bind_group(0, &model.gpu.bind_group, &[]);
        render_pass.draw(0..3, 0..model.triangulation.inner_faces().count() as u32);
    }
    queue.submit(Some(encoder.finish()));

    // let draw = app.draw();

    // // model.triangulation.vertices().for_each(|v| {
    // //     draw.ellipse().xy(v.data().pos).color(BLUE).w_h(15.0, 15.0);
    // // });

    // for face in model.triangulation.inner_faces() {
    //     for edge in face.adjacent_edges() {
    //         let v1 = edge.from().data().pos;
    //         let v2 = edge.to().data().pos;

    //         draw.line().start(v1).end(v2).color(BLACK).weight(2.0);
    //     }
    // }

    // draw.to_frame(app, &frame).unwrap();
}

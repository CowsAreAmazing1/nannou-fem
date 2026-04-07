use nannou::prelude::*;
use spade::{ConstrainedDelaunayTriangulation, HasPosition, Point2, Triangulation};

use crate::fem::{build_mesh, point_in_any_polygon};
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
}

/// Sets up a constrained Delaunay triangulation, with the given vertices constrained in a closed loop, refines, and returns the triangulation.
fn setup_triangulation(
    to_constrain: &[Vec<Vec2>],
    half_width: f32,
    half_height: f32,
    refinement_params: Option<spade::RefinementParameters<f32>>,
) -> ConstrainedDelaunayTriangulation<Vertex> {
    let verts = [
        vec2(-half_width, -half_height),
        vec2(half_width, -half_height),
        vec2(half_width, half_height),
        vec2(-half_width, half_height),
    ];

    let mut triangulation: ConstrainedDelaunayTriangulation<Vertex> =
        ConstrainedDelaunayTriangulation::new();

    for polygon in to_constrain {
        for window in polygon.windows(2) {
            if let [vert1, vert2] = window {
                triangulation
                    .add_constraint_edge(Vertex::from(*vert1), Vertex::from(*vert2))
                    .unwrap();
            }
        }

        if let (Some(first), Some(last)) = (polygon.first(), polygon.last()) {
            triangulation
                .add_constraint_edge(Vertex::from(*last), Vertex::from(*first))
                .unwrap();
        }

        for &vert in &verts {
            triangulation.insert(Vertex::from(vert)).unwrap();
        }
    }

    if let Some(params) = refinement_params {
        triangulation.refine(params);
    } else {
        triangulation
            .refine(spade::RefinementParameters::default().with_max_additional_vertices(10_000));
    };

    triangulation
}

fn model(app: &App) -> Model {
    let win = app.new_window().view(view).maximized(true).build().unwrap();
    let rect = app.window(win).unwrap().rect();
    let (_, _, w, h) = rect.x_y_w_h();
    let half_w = w * 0.5;
    let half_h = h * 0.5;

    let radius = 200.0;
    let num = 100;
    let left_loop = (0..num)
        .map(|i| {
            let angle = map_range(i, 0, num, 0.0, TAU);
            let rad = radius * (1.0 + 0.5 * (4.0 * angle).sin());
            vec2(rad * angle.cos() - 300.0, rad * angle.sin())
        })
        .collect::<Vec<_>>();
    let right_loop = (0..num)
        .map(|i| {
            let angle = map_range(i, 0, num, 0.0, TAU);
            let rad = radius * (1.0 + 0.5 * (4.0 * angle).sin());
            vec2(rad * angle.cos() + 300.0, rad * angle.sin())
        })
        .collect::<Vec<_>>();
    let constrained_loops = vec![left_loop, right_loop];

    let params = spade::RefinementParameters::default().with_max_additional_vertices(10_000);
    // .with_max_allowed_area(100.0)
    // .with_angle_limit(spade::AngleLimit::from_deg(30.0)),

    let triangulation = setup_triangulation(&constrained_loops, half_w, half_h, Some(params));

    let fem_mesh = build_mesh(&triangulation, &constrained_loops);

    let values = fem_mesh
        .positions
        .iter()
        .map(|&p| {
            if point_in_any_polygon(p, &constrained_loops) {
                1.0
            } else {
                0.0
            }
        })
        .collect::<Vec<_>>();

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
    // println!(
    //     "FEM step 2: assembled Poisson system with {} nodes, {} non-zeros, rhs size {}",
    //     poisson.node_count(),
    //     poisson.nnz(),
    //     poisson.rhs.len()
    // );
    // println!(
    //     "FEM step 3: CG converged={}, iterations={}, residual_l2={:.3e}",
    //     solve.converged, solve.iterations, solve.residual_l2
    // );

    Model { triangulation, gpu }
}

fn update(app: &App, model: &mut Model, _update: Update) {
    let window = app.main_window();
    let rect = window.rect();
    let device = window.device();
    let queue = window.queue();

    let (_, _, w, h) = rect.x_y_w_h();
    let half_w = w * 0.5;
    let half_h = h * 0.5;

    let radius = 200.0;
    let num = 100;
    let left_loop = (0..num)
        .map(|i| {
            let angle = map_range(i, 0, num, 0.0, TAU);
            let rad = radius * (1.0 + 0.5 * (4.0 * angle + app.time).sin());
            vec2(rad * angle.cos() - 300.0, rad * angle.sin())
        })
        .collect::<Vec<_>>();
    let right_loop = (0..num)
        .map(|i| {
            let angle = map_range(i, 0, num, 0.0, TAU);
            let rad = radius * (1.0 + 0.5 * (4.0 * angle + app.time).sin());
            vec2(rad * angle.cos() + 300.0, rad * angle.sin())
        })
        .collect::<Vec<_>>();
    let constrained_loops = vec![left_loop, right_loop];

    let params = spade::RefinementParameters::default().with_max_additional_vertices(100_000);
    // .with_max_allowed_area(100.0)
    // .with_angle_limit(spade::AngleLimit::from_deg(30.0)),

    model.triangulation = setup_triangulation(&constrained_loops, half_w, half_h, Some(params));

    let fem_mesh = build_mesh(&model.triangulation, &constrained_loops);
    let values = fem_mesh
        .positions
        .iter()
        .map(|&p| {
            if point_in_any_polygon(p, &constrained_loops) {
                1.0
            } else {
                0.0
            }
        })
        .collect::<Vec<_>>();

    let (vertices, tris) = GpuState::prepare_geometry(&model.triangulation, rect);
    model
        .gpu
        .upload_mesh(device, queue, &vertices, &tris, Some(&values));
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

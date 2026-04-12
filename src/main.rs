use nannou::prelude::*;
use nannou_egui::Egui;
use spade::{ConstrainedDelaunayTriangulation, HasPosition, Point2, Triangulation};
use std::collections::HashMap;

use crate::egui::UiState;
use crate::fem::{Body, FemMesh, LinearSystem};
use crate::gpu::GpuState;
mod egui;
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

/// Allowing Nannou Vec2s to be used as vertices in a triangulation
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

struct Params {
    shape_parameters: [f32; 6], // [polygon resolution, outer radius, inner radius, omega, spin speed, spacing]

    max_additional_vertices: usize,
    max_allowed_area: f32,
    angle_limit: f64,
    refinement_success: Option<bool>,

    num_vertices: Option<usize>,
    draw_triangulation: bool,

    solution_success: Option<bool>,
    solution_steps: u64,

    contour_steps: u32,
    show_contours: bool,
    colors: [[f32; 3]; 4],
}

impl Default for Params {
    fn default() -> Self {
        Self {
            shape_parameters: [100.0, 200.0, 0.5, 4.0, 1.0, 500.0],
            max_additional_vertices: 10_000,
            max_allowed_area: 800.0,
            angle_limit: 30.0,
            refinement_success: None,

            num_vertices: None,
            draw_triangulation: false,

            solution_success: None,
            solution_steps: 100,

            contour_steps: 12,
            show_contours: true,
            colors: [
                [0.05, 0.08, 0.45],
                [0.05, 0.70, 0.95],
                [0.95, 0.85, 0.15],
                [0.85, 0.15, 0.10],
            ],
        }
    }
}

struct Model {
    triangulation: ConstrainedDelaunayTriangulation<Vertex>,
    gpu: GpuState,
    ui: UiState,
    params: Params,
    k_matrix: Option<HashMap<(usize, usize), f32>>,
}

/// Sets up a constrained Delaunay triangulation, with the given vertices constrained in a closed loop, then refines. Returns the triangulation, and a boolean indicating whether the refinement finished without running out of vertices.
fn setup_triangulation<const N: usize>(
    to_constrain: &Vec<[Vec2; N]>,
    half_width: f32,
    half_height: f32,
    refinement_params: Option<spade::RefinementParameters<f32>>,
) -> (ConstrainedDelaunayTriangulation<Vertex>, bool) {
    let verts = [
        vec2(-half_width, -half_height),
        vec2(half_width, -half_height),
        vec2(half_width, half_height),
        vec2(-half_width, half_height),
    ];

    let mut triangulation: ConstrainedDelaunayTriangulation<Vertex> =
        ConstrainedDelaunayTriangulation::new();

    for &vert in &verts {
        triangulation.insert(Vertex::from(vert)).unwrap();
    }

    for polygon in to_constrain {
        triangulation
            .add_constraint_edges(polygon.iter().copied().map(Vertex::from), true)
            .unwrap();
    }

    let result = if let Some(params) = refinement_params {
        triangulation.refine(params)
    } else {
        triangulation
            .refine(spade::RefinementParameters::default().with_max_additional_vertices(10_000))
    };

    (triangulation, result.refinement_complete)
}

fn model(app: &App) -> Model {
    let win = app
        .new_window()
        .msaa_samples(1)
        .raw_event(raw_ui_event)
        .view(view)
        .maximized(true)
        .build()
        .unwrap();
    let rect = app.window(win).unwrap().rect();
    let (_, _, w, h) = rect.x_y_w_h();
    let half_w = w * 0.5;
    let half_h = h * 0.5;

    let params = Params::default();

    const N: usize = 100;

    let bodies = [
        Body::new(1.0, |t| {
            let angle = t * TAU;
            let rad = 200.0 * (1.0 + 0.5 * (4.0 * angle).sin());
            vec2(rad * angle.cos() - 300.0, rad * angle.sin())
        }),
        Body::new(1.0, |t| {
            let angle = t * TAU;
            let rad = 200.0 * (1.0 + 0.5 * (4.0 * angle).sin());
            vec2(rad * angle.cos() + 300.0, rad * angle.sin())
        }),
    ];

    let constrained_loops = bodies.iter().map(|b| b.sample_boundary::<N>()).collect();

    let (triangulation, _) = setup_triangulation(&constrained_loops, half_w, half_h, None);

    let mut fem_mesh = FemMesh::build_mesh(&triangulation, &constrained_loops);
    fem_mesh.compute_density(&constrained_loops, &bodies);

    let window = app.main_window();
    let device = window.device();
    let queue = window.queue();
    let gpu = GpuState::new(
        device,
        queue,
        &triangulation,
        window.rect(),
        Some(fem_mesh.node_density),
    );
    gpu.upload_render_settings(queue, params.contour_steps, params.show_contours);
    gpu.upload_color_map(queue, params.colors);

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

    let egui = Egui::from_window(&window);
    let ui = UiState::new(egui);

    Model {
        triangulation,
        gpu,
        ui,
        params,
        k_matrix: None,
    }
}

fn update(app: &App, model: &mut Model, _update: Update) {
    let window = app.main_window();
    let rect = window.rect();
    let device = window.device();
    let queue = window.queue();

    let (_, _, w, h) = rect.x_y_w_h();
    let half_w = w * 0.5;
    let half_h = h * 0.5;

    // Build polygons
    // let resolution = model.params.shape_parameters[0] as usize;
    let radius_outer = model.params.shape_parameters[1];
    let radius_inner = model.params.shape_parameters[2];
    let omega = model.params.shape_parameters[3];
    let spin_speed = model.params.shape_parameters[4];
    let spacing = model.params.shape_parameters[5];
    let time = app.time;

    let bodies = [
        Body::new(1.0, move |t| {
            let angle = t * TAU;
            let rad =
                radius_outer * (1.0 + radius_inner * (omega * (angle + spin_speed * time)).sin());
            vec2(rad * angle.cos() - spacing, rad * angle.sin())
        }),
        Body::new(1.0, move |t| {
            let angle = t * TAU;
            let rad = radius_outer * (1.0 + radius_inner * (omega * angle).sin());
            vec2(
                rad * (angle - spin_speed * time).cos() + spacing,
                rad * (angle - spin_speed * time).sin(),
            )
        }),
    ];

    const N: usize = 100;
    let constrained_loops = bodies.iter().map(|b| b.sample_boundary::<N>()).collect();

    // Triangulate and refine
    let params = spade::RefinementParameters::default()
        .with_max_additional_vertices(model.params.max_additional_vertices)
        .with_max_allowed_area(model.params.max_allowed_area)
        .with_angle_limit(spade::AngleLimit::from_deg(model.params.angle_limit));

    let (triangulation, refinement_success) =
        setup_triangulation(&constrained_loops, half_w, half_h, Some(params));
    model.triangulation = triangulation;
    model.params.refinement_success = Some(refinement_success);
    model.params.num_vertices = Some(model.triangulation.vertices().count());

    // Build a Mesh
    let mut fem_mesh = FemMesh::build_mesh(&model.triangulation, &constrained_loops);
    fem_mesh.compute_density(&constrained_loops, &bodies);

    let mut dirichlet_values = vec![None; fem_mesh.positions.len()];
    let eps = 1.0e-3;
    for (i, &p) in fem_mesh.positions.iter().enumerate() {
        let on_screen_edge = (p.x - -half_w).abs() < eps
            || (p.x - half_w).abs() < eps
            || (p.y - half_h).abs() < eps
            || (p.y - -half_h).abs() < eps;

        if on_screen_edge {
            dirichlet_values[i] = Some(0.0);
        }
    }

    // Build and solve a linear system
    let ls = LinearSystem::from_mesh(&fem_mesh, &dirichlet_values);
    let (solution, solution_success) = LinearSystem::solve(ls, model.params.solution_steps);
    // model.k_matrix = Some(ls.k);
    model.params.solution_success = Some(solution_success);

    let mut sol = solution.as_slice().to_vec();
    let (mut mn, mut mx) = (f32::INFINITY, f32::NEG_INFINITY);
    for &v in &sol {
        mn = mn.min(v);
        mx = mx.max(v);
    }
    let denom = (mx - mn).max(1e-8);
    for v in &mut sol {
        *v = (*v - mn) / denom;
    }

    let (vertices, tris) = GpuState::prepare_geometry(&model.triangulation, rect);
    model
        .gpu
        .upload_mesh(device, queue, &vertices, &tris, Some(&sol));
    model.gpu.upload_render_settings(
        queue,
        model.params.contour_steps,
        model.params.show_contours,
    );
    model.gpu.upload_color_map(queue, model.params.colors);

    model.ui.update(&mut model.params, model.k_matrix.as_ref());
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

    if model.params.draw_triangulation {
        let draw = app.draw();

        // model.triangulation.vertices().for_each(|v| {
        //     draw.ellipse().xy(v.data().pos).color(BLUE).w_h(15.0, 15.0);
        // });

        for face in model.triangulation.inner_faces() {
            for edge in face.adjacent_edges() {
                let v1 = edge.from().data().pos;
                let v2 = edge.to().data().pos;

                draw.line().start(v1).end(v2).color(BLACK).weight(2.0);
            }
        }

        draw.to_frame(app, &frame).unwrap();
    }

    model.ui.show(&frame);
}

fn raw_ui_event(_app: &App, model: &mut Model, event: &nannou::winit::event::WindowEvent) {
    model.ui.handle_raw_event(event);
}

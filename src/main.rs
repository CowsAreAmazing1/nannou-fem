use nalgebra::DVector;
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
#[derive(Clone)]
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

#[derive(Clone, Copy, PartialEq, Eq)]
enum Visual {
    Density,
    Potential,
    Acceleration,
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum SolveMode {
    FullPerFrame,
    Iterative,
}

#[derive(Clone, Copy, PartialEq)]
struct IterativeSignature {
    shape_parameters: [f32; 6],
    max_additional_vertices: usize,
    max_allowed_area: f32,
    angle_limit: f64,
    half_w: f32,
    half_h: f32,
}

struct IterativeSolveState {
    signature: IterativeSignature,
    triangulation: ConstrainedDelaunayTriangulation<Vertex>,
    mesh: FemMesh,
    linear_system: LinearSystem,
    solution: DVector<f32>,
    total_iterations: u64,
    converged: bool,
}

struct Params {
    visual: Visual,
    solve_mode: SolveMode,

    shape_parameters: [f32; 6], // [resolution, outer radius, inner radius, omega, spin speed, spacing]

    max_additional_vertices: usize,
    max_allowed_area: f32,
    angle_limit: f64,
    refinement_success: Option<bool>,

    num_vertices: Option<usize>,
    draw_triangulation: bool,

    solution_success: Option<bool>,
    solution_steps: u64,
    iterative_steps_per_frame: u64,
    iterative_total_steps: u64,
    iterative_reset_requested: bool,

    contour_steps: u32,
    show_contours: bool,
    colors: [[f32; 3]; 4],
}

impl Default for Params {
    fn default() -> Self {
        Self {
            visual: Visual::Potential,
            solve_mode: SolveMode::FullPerFrame,

            shape_parameters: [100.0, 200.0, 0.5, 4.0, 1.0, 500.0],
            max_additional_vertices: 10_000,
            max_allowed_area: 800.0,
            angle_limit: 30.0,
            refinement_success: None,

            num_vertices: None,
            draw_triangulation: false,

            solution_success: None,
            solution_steps: 100,
            iterative_steps_per_frame: 100,
            iterative_total_steps: 0,
            iterative_reset_requested: false,

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
    last_mode: SolveMode,
    iterative_time_anchor: Option<f32>,
    iterative_state: Option<IterativeSolveState>,
    k_matrix: Option<HashMap<(usize, usize), f32>>,
}

/// Sets up a constrained Delaunay triangulation, with the given vertices constrained in a closed loop, then refines. Returns the triangulation, and a boolean indicating whether the refinement finished without running out of vertices.
fn setup_triangulation(
    to_constrain: &Vec<Vec<Vec2>>,
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
        // .maximized(true)
        .size(1000, 1000)
        .build()
        .unwrap();
    let rect = app.window(win).unwrap().rect();
    let (_, _, w, h) = rect.x_y_w_h();
    let half_w = w * 0.5;
    let half_h = h * 0.5;

    let params = Params::default();

    let bodies = [
        Body::new(1.0, 100, |t| {
            let angle = t * TAU;
            let rad = 200.0 * (1.0 + 0.5 * (4.0 * angle).sin());
            vec2(rad * angle.cos() - 300.0, rad * angle.sin())
        }),
        Body::new(1.0, 100, |t| {
            let angle = t * TAU;
            let rad = 200.0 * (1.0 + 0.5 * (4.0 * angle).sin());
            vec2(rad * angle.cos() + 300.0, rad * angle.sin())
        }),
    ];

    let constrained_loops = bodies.iter().map(|b| b.sample_boundary()).collect();

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

    let egui = Egui::from_window(&window);
    let ui = UiState::new(egui);

    Model {
        triangulation,
        gpu,
        ui,
        last_mode: params.solve_mode,
        iterative_time_anchor: None,
        iterative_state: None,
        params,
        k_matrix: None,
    }
}

fn normalize_vec(v: Vec<f32>) -> Vec<f32> {
    let max = v.iter().cloned().fold(f32::NEG_INFINITY, f32::max);
    let min = v.iter().cloned().fold(f32::INFINITY, f32::min);
    v.into_iter()
        .map(|x| (x - min) / (max - min + 1.0e-6)) // add small epsilon to avoid division by zero
        .collect()
}

/// Compute nodal acceleration magnitude from scalar potential values.
/// For linear triangular elements, grad(phi) is constant per element.
fn node_acceleration_magnitude(mesh: &FemMesh, potential: &[f32]) -> Vec<f32> {
    let n = mesh.positions.len();
    let mut node_acc = vec![vec2(0.0, 0.0); n];
    let mut node_weight = vec![0.0f32; n];

    for &[i0, i1, i2] in &mesh.elements {
        let p0 = mesh.positions[i0];
        let p1 = mesh.positions[i1];
        let p2 = mesh.positions[i2];

        let two_a = (p1.x - p0.x) * (p2.y - p0.y) - (p2.x - p0.x) * (p1.y - p0.y);
        let area = 0.5 * two_a.abs();
        if area <= 1.0e-12 {
            continue;
        }

        let b = [p1.y - p2.y, p2.y - p0.y, p0.y - p1.y];
        let c = [p2.x - p1.x, p0.x - p2.x, p1.x - p0.x];
        let phi = [potential[i0], potential[i1], potential[i2]];

        let mut grad_phi = vec2(0.0, 0.0);
        for k in 0..3 {
            grad_phi += vec2(b[k], c[k]) * (phi[k] / (2.0 * area));
        }

        // a = -grad(phi)
        let a_elem = -grad_phi;
        for &idx in &[i0, i1, i2] {
            node_acc[idx] += a_elem * area;
            node_weight[idx] += area;
        }
    }

    for i in 0..n {
        if node_weight[i] > 0.0 {
            node_acc[i] /= node_weight[i];
        }
    }

    node_acc.into_iter().map(|a| a.length()).collect()
}

fn build_frame_system(
    constrained_loops: &Vec<Vec<Vec2>>,
    bodies: &[Body],
    half_w: f32,
    half_h: f32,
    params: &Params,
) -> (
    ConstrainedDelaunayTriangulation<Vertex>,
    FemMesh,
    LinearSystem,
    bool,
) {
    let refinement_params = spade::RefinementParameters::default()
        .with_max_additional_vertices(params.max_additional_vertices)
        .with_max_allowed_area(params.max_allowed_area)
        .with_angle_limit(spade::AngleLimit::from_deg(params.angle_limit));

    let (triangulation, refinement_success) =
        setup_triangulation(constrained_loops, half_w, half_h, Some(refinement_params));

    let mut fem_mesh = FemMesh::build_mesh(&triangulation, constrained_loops);
    fem_mesh.compute_density(constrained_loops, bodies);
    fem_mesh.dirichlet_values(0.0, half_w, half_h);
    let ls = LinearSystem::from_mesh(&fem_mesh);

    (triangulation, fem_mesh, ls, refinement_success)
}

fn update(app: &App, model: &mut Model, _update: Update) {
    let window = app.main_window();
    let rect = window.rect();
    let device = window.device();
    let queue = window.queue();

    let (_, _, w, h) = rect.x_y_w_h();
    let half_w = w * 0.5;
    let half_h = h * 0.5;

    if model.params.solve_mode != model.last_mode {
        model.iterative_state = None;
        model.iterative_time_anchor = None;
        model.params.iterative_total_steps = 0;
        model.params.iterative_reset_requested = true;
        model.params.solution_success = None;
        model.last_mode = model.params.solve_mode;
    }

    if model.params.solve_mode == SolveMode::FullPerFrame {
        model.iterative_state = None;
        model.iterative_time_anchor = None;
        model.params.iterative_total_steps = 0;
    } else if model.params.iterative_reset_requested {
        model.iterative_state = None;
        model.iterative_time_anchor = Some(app.time);
    }

    // Build polygons
    let resolution = model.params.shape_parameters[0] as usize;
    let radius_outer = model.params.shape_parameters[1];
    let radius_inner = model.params.shape_parameters[2];
    let omega = model.params.shape_parameters[3];
    let angle_offset = model.params.shape_parameters[4];
    let spacing = model.params.shape_parameters[5];

    let bodies = [
        Body::new(1.0, resolution, move |t| {
            let angle = t * TAU - angle_offset;
            let rad = radius_outer * (1.0 + radius_inner * (omega * angle).cos());
            vec2(rad * angle.cos() - spacing, rad * angle.sin())
        }),
        Body::new(1.0, resolution, move |t| {
            let angle = t * TAU - angle_offset;
            let rad = radius_outer * (1.0 + radius_inner * (omega * angle).cos());
            vec2(rad * (angle).cos() + spacing, rad * (angle).sin())
        }),
        // Body::new(1.0, |t| {
        //     vec2(200.0 * (TAU * t).cos(), 200.0 * (TAU * t).sin())
        // }),
        // Body::new(5.0, move |t| {
        //     vec2(500.0 * time.sin(), 500.0 * time.cos())
        //         + vec2(20.0 * (TAU * t).cos(), 20.0 * (TAU * t).sin())
        // }),
    ];

    let constrained_loops = bodies.iter().map(|b| b.sample_boundary()).collect();

    let values = match model.params.solve_mode {
        SolveMode::FullPerFrame => {
            let (triangulation, fem_mesh, ls, refinement_success) =
                build_frame_system(&constrained_loops, &bodies, half_w, half_h, &model.params);

            model.triangulation = triangulation;
            model.params.refinement_success = Some(refinement_success);
            model.params.num_vertices = Some(model.triangulation.vertices().count());

            let (solution, solution_success, _) = ls.solve(model.params.solution_steps);
            model.params.solution_success = Some(solution_success);

            let sol = solution.as_slice().to_vec();
            let acc = node_acceleration_magnitude(&fem_mesh, &sol);

            normalize_vec(match model.params.visual {
                Visual::Density => fem_mesh.node_density,
                Visual::Potential => sol,
                Visual::Acceleration => acc,
            })
        }
        SolveMode::Iterative => {
            let signature = IterativeSignature {
                shape_parameters: model.params.shape_parameters,
                max_additional_vertices: model.params.max_additional_vertices,
                max_allowed_area: model.params.max_allowed_area,
                angle_limit: model.params.angle_limit,
                half_w,
                half_h,
            };

            let needs_rebuild = model.params.iterative_reset_requested
                || model
                    .iterative_state
                    .as_ref()
                    .map(|state| state.signature != signature)
                    .unwrap_or(true);

            if needs_rebuild {
                let (triangulation, mesh, ls, refinement_success) =
                    build_frame_system(&constrained_loops, &bodies, half_w, half_h, &model.params);

                model.params.refinement_success = Some(refinement_success);
                model.params.num_vertices = Some(triangulation.vertices().count());

                model.iterative_state = Some(IterativeSolveState {
                    signature,
                    triangulation,
                    mesh,
                    linear_system: ls,
                    solution: DVector::zeros(0),
                    total_iterations: 0,
                    converged: false,
                });
                model.params.iterative_reset_requested = false;
            }

            let state = model
                .iterative_state
                .as_mut()
                .expect("iterative state must exist when in iterative mode");

            if state.solution.is_empty() {
                state.solution = DVector::zeros(state.mesh.positions.len());
            }

            if !state.converged {
                let (next_solution, converged, step_iterations) =
                    state.linear_system.solve_with_initial_guess(
                        state.solution.clone(),
                        model.params.iterative_steps_per_frame,
                    );
                state.solution = next_solution;
                state.converged = converged;
                state.total_iterations += step_iterations;
            }

            model.triangulation = state.triangulation.clone();
            model.params.solution_success = Some(state.converged);
            model.params.iterative_total_steps = state.total_iterations;

            let sol = state.solution.as_slice().to_vec();
            let acc = node_acceleration_magnitude(&state.mesh, &sol);
            normalize_vec(match model.params.visual {
                Visual::Density => state.mesh.node_density.clone(),
                Visual::Potential => sol,
                Visual::Acceleration => acc,
            })
        }
    };

    let (vertices, tris) = GpuState::prepare_geometry(&model.triangulation, rect);
    model
        .gpu
        .upload_mesh(device, queue, &vertices, &tris, Some(&values));
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

use nannou::prelude::*;
use spade::{ConstrainedDelaunayTriangulation, HasPosition, Point2, Triangulation};

use crate::gpu::GpuState;
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

    triangulation.refine(spade::RefinementParameters::default());

    triangulation
}

fn point_on_segment(p: Vec2, a: Vec2, b: Vec2, eps: f32) -> bool {
    let ab = b - a;
    let ap = p - a;

    let cross = ab.perp_dot(ap).abs();
    if cross > eps {
        return false;
    }

    let dot = ap.dot(ab);
    if dot < -eps {
        return false;
    }

    let ab_len_sq = ab.length_squared();
    if dot > ab_len_sq + eps {
        return false;
    }

    true
}

fn point_in_or_on_polygon(point: Vec2, polygon: &[Vec2]) -> bool {
    if polygon.len() < 3 {
        return false;
    }

    let max_extent = polygon.iter().map(|v| v.length()).fold(1.0_f32, f32::max);
    let eps = 1.0e-4 * max_extent;

    for i in 0..polygon.len() {
        let a = polygon[i];
        let b = polygon[(i + 1) % polygon.len()];
        if point_on_segment(point, a, b, eps) {
            return true;
        }
    }

    // Ray cast to the right; toggles every time we cross an edge.
    let mut inside = false;
    for i in 0..polygon.len() {
        let a = polygon[i];
        let b = polygon[(i + 1) % polygon.len()];

        let intersects = (a.y > point.y) != (b.y > point.y)
            && point.x < (b.x - a.x) * (point.y - a.y) / ((b.y - a.y) + f32::EPSILON) + a.x;

        if intersects {
            inside = !inside;
        }
    }

    inside
}

fn build_values(
    triangulation: &ConstrainedDelaunayTriangulation<Vertex>,
    constrained_loop: &[Vec2],
) -> Vec<f32> {
    triangulation
        .vertices()
        .map(|vert| {
            if point_in_or_on_polygon(vert.data().pos, constrained_loop) {
                1.0
            } else {
                0.0
            }
        })
        .collect()
}

fn model(app: &App) -> Model {
    let win = app.new_window().view(view).maximized(true).build().unwrap();
    let rect = app.window(win).unwrap().rect();
    let (_, _, w, h) = rect.x_y_w_h();
    let half_w = w * 0.5;
    let half_h = h * 0.5;

    let radius = 500.0;
    let num = 100;
    let to_constrain = (0..num)
        .map(|i| {
            let angle = map_range(i, 0, num, 0.0, TAU);
            vec2(radius * angle.cos(), radius * angle.sin())
        })
        .collect::<Vec<_>>();
    // let to_constrain = (0..num)
    //     .map(|i| {
    //         let angle = map_range(i, 0, num, 0.0, TAU);
    //         let rad = radius * (1.0 + 0.25 * (5.0 * angle).sin());
    //         vec2(rad * angle.cos(), rad * angle.sin())
    //     })
    //     .collect::<Vec<_>>();

    let triangulation = setup_triangulation(&to_constrain, half_w, half_h);
    let values = build_values(&triangulation, &to_constrain);

    let binding = app.main_window();
    let device = binding.device();
    let queue = binding.queue();
    let gpu = GpuState::new(device, queue, &triangulation, binding.rect(), Some(&values));

    println!(
        "Triangulation has {} vertices and {} faces",
        triangulation.vertices().count(),
        triangulation.inner_faces().count()
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

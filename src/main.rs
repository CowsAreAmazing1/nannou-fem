use nannou::prelude::*;
use spade::{
    ConstrainedDelaunayTriangulation, HasPosition, Point2, Triangulation,
    handles::FixedVertexHandle,
};

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
    handle: FixedVertexHandle,
    gpu: GpuState,
}

fn model(app: &App) -> Model {
    let win = app.new_window().view(view).maximized(true).build().unwrap();
    let rect = app.window(win).unwrap().rect();
    let (_, _, w, h) = rect.x_y_w_h();
    let half_w = w * 0.5;
    let half_h = h * 0.5;

    let radius = 500.0;
    let num = 1000;
    // let to_constrain = (0..num)
    //     .map(|i| {
    //         let angle = map_range(i, 0, num, 0.0, TAU);
    //         vec2(radius * angle.cos(), radius * angle.sin())
    //     })
    //     .collect::<Vec<_>>();
    // let to_constrain = (0..num)
    //     .map(|i| {
    //         let angle = map_range(i, 0, num, 0.0, TAU);
    //         let rad = radius * (1.0 + 0.25 * (5.0 * angle).sin());
    //         vec2(rad * angle.cos(), rad * angle.sin())
    //     })
    //     .collect::<Vec<_>>();
    let to_constrain = (0..num)
        .map(|i| {
            let angle = map_range(i, 0, num, 0.0, 10.0 * TAU);
            vec2(
                radius * angle * 0.1 * angle.cos(),
                radius * angle * 0.1 * angle.sin(),
            )
        })
        .collect::<Vec<_>>();

    // let verts = (0..10)
    //     .map(|_| vec2(random_range(-half_w, half_w), random_range(-half_h, half_h)))
    //     .collect::<Vec<_>>();
    let verts = [
        vec2(-half_w, -half_h),
        vec2(half_w, -half_h),
        vec2(half_w, half_h),
        vec2(-half_w, half_h),
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

    verts.iter().for_each(|&vert| {
        triangulation.insert(Vertex::from(vert)).unwrap();
    });

    triangulation.refine(spade::RefinementParameters::default());

    let handle = triangulation.insert(Vertex::from(Vec2::ZERO)).unwrap();

    let binding = app.main_window();
    let device = binding.device();
    let queue = binding.queue();
    let gpu = GpuState::new(device, queue, &triangulation, binding.rect());

    println!("faces: {:?}", &triangulation.inner_faces().count());

    Model {
        triangulation,
        handle,
        gpu,
    }
}

fn update(app: &App, model: &mut Model, _update: Update) {
    model.triangulation.remove(model.handle);

    let new_handle = model
        .triangulation
        .insert(app.mouse.position().into())
        .unwrap();

    model.handle = new_handle;

    // model
    //     .triangulation
    //     .refine(spade::RefinementParameters::default());

    let window = app.main_window();
    let device = window.device();
    let queue = window.queue();

    let (vertices, tris, _values) = GpuState::prepare_buffers(&model.triangulation, window.rect());

    model.gpu.upload_mesh(device, queue, &vertices, &tris, None);
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

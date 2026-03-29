use nannou::prelude::*;
use spade::{
    DelaunayTriangulation, HasPosition, Point2, Triangulation, handles::FixedVertexHandle,
};

use crate::gpu::GpuState;
mod gpu;

fn main() {
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

impl HasPosition for Vertex {
    type Scalar = f32;

    fn position(&self) -> Point2<Self::Scalar> {
        spade::Point2::new(self.pos.x, self.pos.y)
    }
}

struct Model {
    triangulation: DelaunayTriangulation<Vertex>,
    handle: FixedVertexHandle,
    gpu: GpuState,
}

fn model(app: &App) -> Model {
    let win = app
        .new_window()
        .view(view)
        .maximized(false)
        .build()
        .unwrap();
    let rect = app.window(win).unwrap().rect();
    let (_, _, w, h) = rect.x_y_w_h();
    let half_w = w * 0.5;
    let half_h = h * 0.5;

    let verts = (0..1_000_000)
        .map(|_| vec2(random_range(-half_w, half_w), random_range(-half_h, half_h)))
        .collect::<Vec<_>>();

    let mut triangulation: DelaunayTriangulation<_> = DelaunayTriangulation::new();

    verts.iter().for_each(|&vert| {
        triangulation.insert(Vertex::from(vert)).unwrap();
    });

    let handle = triangulation.insert(Vertex::from(Vec2::ZERO)).unwrap();

    let gpu = GpuState::new(app.main_window().device(), &triangulation, [half_w, half_h]);

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

    let window = app.main_window();
    let queue = window.queue();
    let (_, _, w, h) = window.rect().x_y_w_h();
    let half_w = w * 0.5;
    let half_h = h * 0.5;

    let (vertices, tris, values) =
        GpuState::prepare_buffers(&model.triangulation, [half_w, half_h]);

    queue.write_buffer(&model.gpu.vertex_buffer, 0, bytemuck::cast_slice(&vertices));
    queue.write_buffer(&model.gpu.tri_buffer, 0, bytemuck::cast_slice(&tris));
    queue.write_buffer(&model.gpu.values_buffer, 0, bytemuck::cast_slice(&values));
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

    // draw.background().color(WHITE);

    // model.triangulation.vertices().for_each(|v| {
    //     draw.ellipse().xy(v.data().pos).color(BLUE).w_h(15.0, 15.0);
    // });

    // for face in model.triangulation.inner_faces() {
    //     for edge in face.adjacent_edges() {
    //         let v1 = edge.from().data().pos;
    //         let v2 = edge.to().data().pos;

    //         draw.line().start(v1).end(v2).color(BLACK).weight(2.0);
    //     }
    // }

    // draw.to_frame(app, &frame).unwrap();
}

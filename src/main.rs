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
    handle: Option<FixedVertexHandle>,
    gpu: GpuState,
}

fn model(app: &App) -> Model {
    let win = app.new_window().view(view).maximized(true).build().unwrap();
    let rect = app.window(win).unwrap().rect();
    let (_, _, w, h) = rect.x_y_w_h();

    let verts = (0..100)
        .map(|_| vec2(random_range(-w, w), random_range(-h, h)))
        .collect::<Vec<_>>();

    let mut triangulation: DelaunayTriangulation<_> = DelaunayTriangulation::new();

    verts.iter().for_each(|&vert| {
        triangulation.insert(Vertex::from(vert)).unwrap();
    });

    let gpu = GpuState::new(app.main_window().device(), &triangulation, [w, h]);

    Model {
        triangulation,
        handle: None,
        gpu,
    }
}

fn update(app: &App, model: &mut Model, _update: Update) {
    if let Some(handle) = model.handle {
        model.triangulation.remove(handle);
    }

    let new_handle = model
        .triangulation
        .insert(app.mouse.position().into())
        .unwrap();

    model.handle = Some(new_handle);
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
        println!("{:?}", model.triangulation.inner_faces().count());
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

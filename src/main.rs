use nannou::prelude::*;
use spade::{
    DelaunayTriangulation, HasPosition, Point2, Triangulation, handles::FixedVertexHandle,
};

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
}

fn model(app: &App) -> Model {
    let win = app.new_window().view(view).build().unwrap();
    let rect = app.window(win).unwrap().rect();
    let (_, _, w, h) = rect.x_y_w_h();

    let verts = (0..100)
        .map(|_| vec2(random_range(-w, w), random_range(-h, h)))
        .collect::<Vec<_>>();

    let mut triangulation: DelaunayTriangulation<_> = DelaunayTriangulation::new();

    verts.iter().for_each(|&vert| {
        triangulation.insert(Vertex::from(vert)).unwrap();
    });

    Model {
        triangulation,
        handle: None,
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
    let draw = app.draw();

    draw.background().color(WHITE);

    model.triangulation.vertices().for_each(|v| {
        draw.ellipse().xy(v.data().pos).color(BLUE).w_h(15.0, 15.0);
    });

    for face in model.triangulation.inner_faces() {
        for edge in face.adjacent_edges() {
            let v1 = edge.from().data().pos;
            let v2 = edge.to().data().pos;

            draw.line().start(v1).end(v2).color(BLACK).weight(2.0);
        }
    }

    draw.to_frame(app, &frame).unwrap();
}

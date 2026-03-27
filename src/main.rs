use nannou::prelude::*;
use triangulate::{ListFormat, Polygon, PolygonList, Vertex, formats};

fn main() {
    nannou::app(model)
        .update(update)
        .loop_mode(LoopMode::refresh_sync())
        .run();
}

struct Model {
    verts: Vec<Vec2>,
    triangulated_indices: Option<Vec<[usize; 2]>>,
    polygons: Option<Vec<Vec<[f32; 2]>>>,
}

fn model(app: &App) -> Model {
    let win = app.new_window().view(view).build().unwrap();
    let rect = app.window(win).unwrap().rect();
    let (_, _, w, h) = rect.x_y_w_h();

    let verts = (0..10)
        .map(|_| vec2(random_range(-w, w), random_range(-h, h)))
        .collect::<Vec<_>>();

    Model {
        verts,
        triangulated_indices: None,
        polygons: None,
    }
}

fn update(_app: &App, model: &mut Model, _update: Update) {
    let mut triangulated_indices = Vec::<[usize; 2]>::new();

    let polygons: Vec<_> = model.verts.iter().map(|&v| [v.x, v.y]).collect();

    polygons.triangulate(formats::DeindexedListFormat::new(triangulated_indices));

    // polygons
    //     .triangulate(formats::IndexedListFormat::new(&mut triangulated_indices).into_fan_format())
    //     .expect("Triangulation failed");
    // println!(
    //     "First triangle: {:?}, {:?}, {:?}",
    //     polygons.get_vertex(triangulated_indices[0]),
    //     polygons.get_vertex(triangulated_indices[1]),
    //     polygons.get_vertex(triangulated_indices[2])
    // );
    // println!("{:?}", polygons);
    // println!("{:?}", polygons);
}

fn view(app: &App, model: &Model, frame: Frame) {
    let draw = app.draw().translate(vec3(-150.0, -150.0, 0.0));

    draw.background().color(WHITE);

    model.verts.iter().for_each(|v| {
        draw.ellipse()
            .x_y(300.0 * v.x, 300.0 * v.y)
            .w_h(10.0, 10.0)
            .color(SLATEBLUE);
    });

    // draw.polygon()
    //     .points(model.triangulated_indices.chunks(3).map(|triangle| {
    //         triangle.iter().map(|&idx| {
    //             let vertex = model.polygons.get_vertex(idx);
    //             pt2(300.0 * vertex.x(), 300.0 * vertex.y())
    //         })
    //     }))
    //     .color(STEELBLUE);

    model.triangulated_indices.chunks(3).for_each(|chk| {
        let points = chk.iter().map(|&idx| *model.polygons.get_vertex(idx));
        let points = points.map(|v| pt2(300.0 * v.x(), 300.0 * v.y()));
        draw.polyline().points(points);
    });

    draw.to_frame(app, &frame).unwrap();
}

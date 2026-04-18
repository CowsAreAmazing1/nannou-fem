use fem::{
    app::Params,
    egui::UiState,
    fem::{Vertex, setup_triangulation},
    objects::{Body, ConstraintHandler, PortalPair, Sampleable},
};
use nannou::prelude::*;
use nannou_egui::Egui;
use spade::{ConstrainedDelaunayTriangulation, Triangulation};

fn main() {
    nannou::app(model)
        .update(update)
        .loop_mode(LoopMode::refresh_sync())
        .run();
}

struct Model {
    triangulation: ConstrainedDelaunayTriangulation<Vertex>,
    ui: UiState,
    params: Params,
    portals: Option<PortalPair>,
}

fn model(app: &App) -> Model {
    let win = app
        .new_window()
        .msaa_samples(1)
        .raw_event(raw_ui_event)
        .view(view)
        .maximized(true)
        .size(1000, 1000)
        .build()
        .unwrap();
    let window = app.window(win).unwrap();
    let rect = window.rect();
    let (_, _, w, h) = rect.x_y_w_h();
    let half_w = w * 0.5;
    let half_h = h * 0.5;

    let params = Params::default();

    let constraints = ConstraintHandler::new().add_object(Body::new(1.0, 100, |t| {
        let angle = t * TAU;
        vec2(100.0 * angle.cos(), 100.0 * angle.sin())
    }));

    let (triangulation, _) = setup_triangulation(&constraints, half_w, half_h, None);

    let egui = Egui::from_window(&window);
    let ui = UiState::new(egui);

    Model {
        triangulation,
        ui,
        params,
        portals: None,
    }
}

fn update(app: &App, model: &mut Model, _update: Update) {
    let window = app.main_window();
    let rect = window.rect();

    let (_, _, w, h) = rect.x_y_w_h();
    let half_w = w * 0.5;
    let half_h = h * 0.5;

    // Build polygons
    // let resolution = model.params.shape_parameters[0] as usize;
    // let radius_outer = model.params.shape_parameters[1];
    // let radius_inner = model.params.shape_parameters[2];
    // let omega = model.params.shape_parameters[3];
    // let angle_offset = model.params.shape_parameters[4];
    // let spacing = model.params.shape_parameters[5];

    let pp = PortalPair::from_positions(
        vec2(-200.0, 400.0),
        vec2(200.0, 400.0),
        vec2(-200.0, -400.0),
        vec2(200.0, -400.0),
    );
    model.portals = Some(pp.clone());

    let constraints = ConstraintHandler::new()
        // .add_object(Body::new(1.0, resolution, |t| {
        //     vec2(200.0 * (TAU * t).cos(), 200.0 * (TAU * t).sin())
        // }))
        .add_object(pp);

    let refinement_params = spade::RefinementParameters::default()
        .with_max_additional_vertices(model.params.max_additional_vertices)
        .with_max_allowed_area(model.params.max_allowed_area);
    // .with_angle_limit(spade::AngleLimit::from_deg(model.params.angle_limit));

    model.triangulation =
        setup_triangulation(&constraints, half_w, half_h, Some(refinement_params)).0;

    model.ui.update(&mut model.params, None);
}

fn view(app: &App, model: &Model, frame: Frame) {
    let draw = app.draw();
    draw.background().color(WHITE);

    model
        .triangulation
        .vertices()
        .enumerate()
        .for_each(|(i, v)| {
            draw.ellipse()
                .xy(v.data().vec2())
                .color(BLUE)
                .w_h(15.0, 15.0);
            draw.text(&i.to_string())
                .xy(v.data().vec2() + vec2(20.0, 10.0))
                .color(BLACK)
                .font_size(20);
        });

    for face in model.triangulation.inner_faces() {
        for edge in face.adjacent_edges() {
            let v1 = edge.from().data().vec2();
            let v2 = edge.to().data().vec2();

            draw.line().start(v1).end(v2).color(BLACK).weight(1.0);
        }
    }

    if let Some(pair) = &model.portals {
        pair.sample().iter().for_each(|portal| {
            draw.polyline().weight(3.0).points(portal.clone());
        });
    }

    draw.to_frame(app, &frame).unwrap();

    model.ui.show(&frame);
}

fn raw_ui_event(_app: &App, model: &mut Model, event: &nannou::winit::event::WindowEvent) {
    model.ui.handle_raw_event(event);
}

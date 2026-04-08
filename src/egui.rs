use nannou_egui::{self, Egui, egui};

use nannou_egui::egui::{Label, Response, RichText, Slider, Style, Ui, Widget};

pub struct StyledSlider<'a> {
    title: String,
    min: f32,
    max: f32,
    value: &'a mut f32,

    style: Option<&'a Style>,
}

impl<'a> StyledSlider<'a> {
    pub fn new(title: impl Into<String>, min: f32, max: f32, value: &'a mut f32) -> Self {
        Self {
            min,
            max,
            value,
            title: title.into(),
            style: None,
        }
    }
}

impl<'a> Widget for StyledSlider<'a> {
    fn ui(self, ui: &mut Ui) -> Response {
        let mut result = None;

        ui.vertical(|ui| {
            ui.horizontal(|ui| {
                let space = (ui.available_width() - 30.0, ui.available_height());
                let text = RichText::new(&self.title).monospace();
                ui.add_sized(space, Label::new(text));
            });

            ui.horizontal(|ui| {
                if let Some(style) = self.style {
                    *ui.style_mut() = style.clone();
                }
                ui.style_mut().spacing.slider_width = ui.available_width() - 70.0;

                let range = self.min..=self.max;
                let slider = Slider::new(self.value, range).logarithmic(true); // .show_value(false).integer();
                result = Some(ui.add(slider));
            })
        });

        result.unwrap()
    }
}

pub struct UiState {
    ui: Egui,
}

impl UiState {
    pub fn new(ui: Egui) -> Self {
        Self { ui }
    }

    pub fn handle_raw_event(&mut self, event: &nannou::winit::event::WindowEvent) {
        self.ui.handle_raw_event(event);
    }

    pub fn update(&mut self, params: &mut crate::Params) {
        let ctx = self.ui.begin_frame();
        egui::Window::new("Controls").show(&ctx, |ui| {
            ui.add(
                egui::Slider::new(&mut params.shape_parameters[0], 2.0..=10_000.0)
                    .integer()
                    .logarithmic(true)
                    .text("Polygon Resolution"),
            );
            ui.add(
                egui::Slider::new(&mut params.shape_parameters[1], 0.0..=400.0)
                    .text("Outer Radius"),
            );
            ui.add(
                egui::Slider::new(&mut params.shape_parameters[2], 0.0..=1.0).text("Inner Radius"),
            );
            ui.add(egui::Slider::new(&mut params.shape_parameters[3], 0.0..=10.0).text("Omega"));
            ui.add(
                egui::Slider::new(&mut params.shape_parameters[4], -2.0..=2.0).text("Spin Speed"),
            );
            ui.add(egui::Slider::new(&mut params.shape_parameters[5], 0.0..=600.0).text("Spacing"));

            ui.separator();

            ui.add(
                egui::Slider::new(&mut params.max_additional_vertices, 0..=100_000)
                    .text("Max Additional Vertices"),
            );
            ui.add(StyledSlider::new(
                "Max Allowed Area",
                1.0,
                10000.0,
                &mut params.max_allowed_area,
            ));
            ui.add(egui::Slider::new(&mut params.angle_limit, 0.0..=40.0).text("Angle Limit"));

            ui.separator();

            let text = if let Some(success) = params.refinement_success {
                if success {
                    "Refinement successful!"
                } else {
                    "Refinement failed."
                }
            } else {
                "Refinement not yet attempted."
            };
            let color = if let Some(success) = params.refinement_success {
                if success {
                    egui::Color32::GREEN
                } else {
                    egui::Color32::RED
                }
            } else {
                egui::Color32::YELLOW
            };
            ui.label(RichText::new(text).color(color));
            let text = if let Some(num_vertices) = params.num_vertices {
                format!("Number of vertices: {}", num_vertices)
            } else {
                "Number of vertices: N/A".to_string()
            };
            ui.label(text);

            ui.separator();

            ui.checkbox(&mut params.draw_triangulation, "Draw Triangulation")
        });
    }

    pub fn show(&self, frame: &nannou::frame::Frame) {
        self.ui.draw_to_frame(frame).unwrap();
    }
}

use std::collections::HashMap;

use egui_plot::{Line, Plot};
use nannou_egui::color_picker::color_edit_button_rgb;
use nannou_egui::{Egui, egui};

use crate::app::{Params, SolveMode, Visual};

#[derive(Clone, Copy, PartialEq, Eq)]
enum MatrixColorMode {
    Density,
    ValueSum,
    AbsValueSum,
}

pub struct StyledSlider<'a> {
    title: String,
    min: f32,
    max: f32,
    value: &'a mut f32,

    style: Option<&'a egui::Style>,
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

impl<'a> egui::Widget for StyledSlider<'a> {
    fn ui(self, ui: &mut egui::Ui) -> egui::Response {
        let mut result = None;

        ui.vertical(|ui| {
            ui.horizontal(|ui| {
                let space = (ui.available_width() - 30.0, ui.available_height());
                let text = egui::RichText::new(&self.title).monospace();
                ui.add_sized(space, egui::Label::new(text));
            });

            ui.horizontal(|ui| {
                if let Some(style) = self.style {
                    *ui.style_mut() = style.clone();
                }
                ui.style_mut().spacing.slider_width = ui.available_width() - 70.0;

                let range = self.min..=self.max;
                let slider = egui::Slider::new(self.value, range).logarithmic(true); // .show_value(false).integer();
                result = Some(ui.add(slider));
            })
        });

        result.unwrap()
    }
}

pub struct UiState {
    ui: Egui,
    matrix_color_mode: MatrixColorMode,
}

impl UiState {
    pub fn new(ui: Egui) -> Self {
        Self {
            ui,
            matrix_color_mode: MatrixColorMode::Density,
        }
    }

    pub fn handle_raw_event(&mut self, event: &nannou::winit::event::WindowEvent) {
        self.ui.handle_raw_event(event);
    }

    pub fn update(&mut self, params: &mut Params, k_matrix: Option<&HashMap<(usize, usize), f32>>) {
        let (ui_state, matrix_color_mode) = (&mut self.ui, &mut self.matrix_color_mode);
        let ctx = ui_state.begin_frame();
        egui::Window::new("Controls").show(&ctx, |ui| {
            ui.horizontal(|ui| {
                ui.label("Visual:");
                ui.selectable_value(&mut params.visual, Visual::Density, "Density");
                ui.selectable_value(&mut params.visual, Visual::Potential, "Potential");
                ui.selectable_value(&mut params.visual, Visual::Acceleration, "Acceleration");
            });

            ui.horizontal(|ui| {
                ui.label("Solver Mode:");
                ui.selectable_value(
                    &mut params.solve_mode,
                    SolveMode::FullPerFrame,
                    "Full Per Frame",
                );
                ui.selectable_value(&mut params.solve_mode, SolveMode::Iterative, "Iterative");
            });

            ui.separator();

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
                egui::Slider::new(&mut params.max_additional_vertices, 0..=1_000_000)
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
            ui.label(egui::RichText::new(text).color(color));
            let text = if let Some(num_vertices) = params.num_vertices {
                format!("Number of vertices: {}", num_vertices)
            } else {
                "Number of vertices: N/A".to_string()
            };
            ui.label(text);

            ui.separator();

            ui.checkbox(&mut params.draw_triangulation, "Draw Triangulation");

            ui.separator();

            if params.solve_mode == SolveMode::Iterative {
                ui.add(
                    egui::Slider::new(&mut params.iterative_steps_per_frame, 1..=10_000)
                        .logarithmic(true)
                        .text("Iterative Steps / Frame"),
                );
                ui.label(format!(
                    "Iterative total steps: {}",
                    params.iterative_total_steps
                ));
                if ui.button("Reset Iterative Solve").clicked() {
                    params.iterative_reset_requested = true;
                    params.iterative_total_steps = 0;
                }

                // Cost plot
                let points: Vec<[f64; 2]> = params
                    .iterative_cost_data
                    .iter()
                    .map(|&(iter, cost)| [iter as f64, (cost as f64).ln()])
                    .collect();

                let line = Line::new(points).name("Cost");

                let plot = Plot::new("Cost plot").view_aspect(3.0);
                plot.show(ui, |plot_ui| plot_ui.line(line));
            } else {
                ui.add(
                    egui::Slider::new(&mut params.solution_steps, 1..=10_000)
                        .logarithmic(true)
                        .text("Full Solve Steps"),
                );
            }

            ui.checkbox(&mut params.show_contours, "Show Contours");
            ui.add(
                egui::Slider::new(&mut params.contour_steps, 2..=100)
                    .logarithmic(true)
                    .text("Contour Steps"),
            );

            ui.horizontal(|ui| {
                color_edit_button_rgb(ui, &mut params.colors[0]);
                color_edit_button_rgb(ui, &mut params.colors[1]);
                color_edit_button_rgb(ui, &mut params.colors[2]);
                color_edit_button_rgb(ui, &mut params.colors[3]);
            });
        });

        if let Some(k_matrix) = k_matrix {
            Self::matrix_window(&ctx, k_matrix, matrix_color_mode);
        }
    }

    fn matrix_window(
        ctx: &egui::Context,
        k_matrix: &HashMap<(usize, usize), f32>,
        matrix_color_mode: &mut MatrixColorMode,
    ) {
        egui::Window::new("K Matrix Structure").show(ctx, |ui| {
            let Some(&(max_row, _)) = k_matrix.keys().max_by_key(|(r, _)| r) else {
                ui.label("K matrix is empty.");
                return;
            };
            let Some(&(_, max_col)) = k_matrix.keys().max_by_key(|(_, c)| c) else {
                ui.label("K matrix is empty.");
                return;
            };

            let rows = max_row + 1;
            let cols = max_col + 1;
            let nnz = k_matrix.len();
            let total_cells = rows.saturating_mul(cols).max(1);
            let density_pct = (nnz as f32 / total_cells as f32) * 100.0;

            ui.label(format!(
                "{}x{} | non-zero entries: {} | density: {:.4}%",
                rows, cols, nnz, density_pct
            ));

            ui.horizontal(|ui| {
                ui.label("Color Mode:");
                ui.selectable_value(matrix_color_mode, MatrixColorMode::Density, "Density");
                ui.selectable_value(matrix_color_mode, MatrixColorMode::ValueSum, "Value Sum");
                ui.selectable_value(
                    matrix_color_mode,
                    MatrixColorMode::AbsValueSum,
                    "Abs Value Sum",
                );
            });

            // Cap the displayed grid size and aggregate large matrices into buckets.
            let max_cells_per_axis = 120usize;
            let bucket_rows = rows.min(max_cells_per_axis);
            let bucket_cols = cols.min(max_cells_per_axis);
            let row_stride = rows.div_ceil(bucket_rows).max(1);
            let col_stride = cols.div_ceil(bucket_cols).max(1);

            let mut buckets = vec![0u32; bucket_rows * bucket_cols];
            let mut value_sums = vec![0.0f32; bucket_rows * bucket_cols];
            let mut abs_value_sums = vec![0.0f32; bucket_rows * bucket_cols];
            for (&(row, col), &value) in k_matrix {
                let bucket_row = (row / row_stride).min(bucket_rows - 1);
                let bucket_col = (col / col_stride).min(bucket_cols - 1);
                let idx = bucket_row * bucket_cols + bucket_col;
                buckets[idx] += 1;
                value_sums[idx] += value;
                abs_value_sums[idx] += value.abs();
            }

            let max_bucket = buckets.iter().copied().max().unwrap_or(1) as f32;
            let max_abs_sum = value_sums
                .iter()
                .copied()
                .map(f32::abs)
                .fold(0.0f32, f32::max)
                .max(1.0e-12);
            let max_abs_value_sum = abs_value_sums
                .iter()
                .copied()
                .fold(0.0f32, f32::max)
                .max(1.0e-12);
            let cell_size = 6.0;
            let canvas_size = egui::vec2(
                bucket_cols as f32 * cell_size,
                bucket_rows as f32 * cell_size,
            );

            egui::ScrollArea::both().show(ui, |ui| {
                let (rect, _) = ui.allocate_exact_size(canvas_size, egui::Sense::hover());
                let painter = ui.painter_at(rect);

                for bucket_row in 0..bucket_rows {
                    for bucket_col in 0..bucket_cols {
                        let idx = bucket_row * bucket_cols + bucket_col;
                        let count = buckets[idx] as f32;
                        let bucket_sum = value_sums[idx];
                        let bucket_abs_sum = abs_value_sums[idx];

                        let color = match *matrix_color_mode {
                            MatrixColorMode::Density => {
                                let intensity = if count <= 0.0 {
                                    0.0
                                } else {
                                    // sqrt improves contrast for sparse patterns.
                                    (count / max_bucket).sqrt()
                                };

                                if count <= 0.0 {
                                    egui::Color32::from_gray(245)
                                } else {
                                    let green = (70.0 + 180.0 * intensity) as u8;
                                    egui::Color32::from_rgb(20, green, 40)
                                }
                            }
                            MatrixColorMode::ValueSum => {
                                let intensity = (bucket_sum.abs() / max_abs_sum).sqrt();
                                if intensity <= 0.0001 {
                                    egui::Color32::from_gray(245)
                                } else if bucket_sum >= 0.0 {
                                    let green = (70.0 + 180.0 * intensity) as u8;
                                    egui::Color32::from_rgb(20, green, 40)
                                } else {
                                    let red = (70.0 + 180.0 * intensity) as u8;
                                    egui::Color32::from_rgb(red, 30, 30)
                                }
                            }
                            MatrixColorMode::AbsValueSum => {
                                let intensity = (bucket_abs_sum / max_abs_value_sum).sqrt();
                                if intensity <= 0.0001 {
                                    egui::Color32::from_gray(245)
                                } else {
                                    // Continuous 3-stop gradient: blue -> yellow -> red.
                                    let (r, g, b) = if intensity < 0.5 {
                                        let t = intensity / 0.5;
                                        let r = (20.0 + (255.0 - 20.0) * t) as u8;
                                        let g = (70.0 + (220.0 - 70.0) * t) as u8;
                                        let b = (170.0 + (30.0 - 170.0) * t) as u8;
                                        (r, g, b)
                                    } else {
                                        let t = (intensity - 0.5) / 0.5;
                                        let r = (255.0 + (200.0 - 255.0) * t) as u8;
                                        let g = (220.0 + (30.0 - 220.0) * t) as u8;
                                        let b = (30.0) as u8;
                                        (r, g, b)
                                    };
                                    egui::Color32::from_rgb(r, g, b)
                                }
                            }
                        };

                        let x = rect.min.x + bucket_col as f32 * cell_size;
                        let y = rect.min.y + bucket_row as f32 * cell_size;
                        let cell_rect = egui::Rect::from_min_size(
                            egui::pos2(x, y),
                            egui::vec2(cell_size - 1.0, cell_size - 1.0),
                        );
                        painter.rect_filled(cell_rect, 0.0, color);
                    }
                }
            });

            if bucket_rows < rows || bucket_cols < cols {
                ui.small(format!(
                    "Downsampled to {}x{} buckets (each cell covers up to {}x{} matrix entries).",
                    bucket_rows, bucket_cols, row_stride, col_stride
                ));
            }

            ui.horizontal(|ui| match *matrix_color_mode {
                MatrixColorMode::Density => {
                    ui.colored_label(egui::Color32::from_gray(245), "Empty");
                    ui.colored_label(egui::Color32::from_rgb(20, 120, 40), "Sparse");
                    ui.colored_label(egui::Color32::from_rgb(20, 250, 40), "Dense");
                }
                MatrixColorMode::ValueSum => {
                    ui.colored_label(egui::Color32::from_rgb(200, 30, 30), "Negative sum");
                    ui.colored_label(egui::Color32::from_gray(245), "Near zero sum");
                    ui.colored_label(egui::Color32::from_rgb(20, 200, 40), "Positive sum");
                }
                MatrixColorMode::AbsValueSum => {
                    ui.colored_label(egui::Color32::from_gray(245), "Low |sum(value)|");
                    ui.colored_label(egui::Color32::from_rgb(255, 220, 30), "Medium");
                    ui.colored_label(egui::Color32::from_rgb(200, 30, 30), "High |sum(value)|");
                }
            });
        });
    }

    pub fn show(&self, frame: &nannou::frame::Frame) {
        self.ui.draw_to_frame(frame).unwrap();
    }
}

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum Visual {
    Density,
    Potential,
    Acceleration,
}

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum SolveMode {
    FullPerFrame,
    Iterative,
}

pub struct Params {
    pub visual: Visual,
    pub solve_mode: SolveMode,

    pub shape_parameters: [f32; 6], // [resolution, outer radius, inner radius, omega, spin speed, spacing]

    pub max_additional_vertices: usize,
    pub max_allowed_area: f32,
    pub angle_limit: f64,
    pub refinement_success: Option<bool>,

    pub num_vertices: Option<usize>,
    pub draw_triangulation: bool,

    pub solution_success: Option<bool>,
    pub solution_steps: u64,
    pub iterative_steps_per_frame: u64,
    pub iterative_total_steps: u64,
    pub iterative_reset_requested: bool,
    pub iterative_cost_data: Vec<(u32, f32)>, // (iteration, cost)

    pub contour_steps: u32,
    pub show_contours: bool,
    pub colors: [[f32; 3]; 4],
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
            iterative_cost_data: Vec::new(),

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

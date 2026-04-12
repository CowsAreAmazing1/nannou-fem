use argmin::{
    core::{Executor, Operator, State},
    solver::conjugategradient::ConjugateGradient,
};
use nalgebra::DVector;
use nannou::prelude::*;
use spade::{ConstrainedDelaunayTriangulation, HasPosition, Triangulation};
use sprs::{CsMat, TriMat};

pub struct Body {
    generator: Box<dyn Fn(f32) -> Vec2>,
    density: f32,
}

impl Body {
    pub fn new(density: f32, generator: impl Fn(f32) -> Vec2 + 'static) -> Self {
        Self {
            generator: Box::new(generator),
            density,
        }
    }

    pub fn sample_boundary<const N: usize>(&self) -> [Vec2; N] {
        std::array::from_fn(|i| {
            // Keep samples on [0, 1) so first/last vertices are distinct.
            let t = i as f32 / N as f32;
            (self.generator)(t)
        })
    }
}

/// Checks if `point` is on the line segment defined by points `a` and `b`, within a certain tolerance `eps`.
fn point_on_segment(point: Vec2, a: Vec2, b: Vec2, eps: f32) -> bool {
    let ab = b - a;
    let ap = point - a;

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

/// Checks if `point` is inside or on the boundary of `polygon` defined by a list of vertices, by first checking if it's on any edge (using `point_on_segment`), and if not, using the ray-casting algorithm to count edge crossings.
pub fn point_in_or_on_polygon(point: Vec2, polygon: &[Vec2]) -> bool {
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

pub fn _point_in_any_polygon(point: Vec2, polygons: &[Vec<Vec2>]) -> bool {
    for polygon in polygons {
        if point_in_or_on_polygon(point, polygon) {
            return true;
        }
    }
    false
}

#[derive(Debug, Clone)]
pub struct FemMesh {
    pub positions: Vec<Vec2>,
    pub elements: Vec<[usize; 3]>,
    pub is_boundary: Vec<bool>,
    pub element_density: Vec<f32>,
    pub node_density: Vec<f32>,
}

impl FemMesh {
    /// Converts a `ConstrainedDelaunayTriangulation` to a `FemMesh`, an easy-to-use format for finite element assembly and solving. The `is_boundary` field is determined by checking if each vertex lies on any of the constrained loops.
    pub fn build_mesh<V, const N: usize>(
        triangulation: &ConstrainedDelaunayTriangulation<V>,
        constrained_loops: &Vec<[Vec2; N]>,
    ) -> FemMesh
    where
        V: HasPosition<Scalar = f32>,
    {
        let positions = triangulation
            .vertices()
            .map(|v| {
                let p = v.position();
                vec2(p.x, p.y)
            })
            .collect::<Vec<_>>();

        let elements = triangulation
            .inner_faces()
            .map(|f| {
                let verts = f.vertices();
                [verts[0].index(), verts[1].index(), verts[2].index()]
            })
            .collect::<Vec<_>>();

        let max_extent = constrained_loops
            .iter()
            .flat_map(|loop_pts| loop_pts.iter())
            .map(|v| v.length())
            .fold(1.0_f32, f32::max);

        let eps = 1.0e-4 * max_extent;
        let is_boundary = positions
            .iter()
            .map(|&p| {
                if constrained_loops.is_empty() {
                    return false;
                }

                for loop_pts in constrained_loops {
                    if loop_pts.len() < 2 {
                        continue;
                    }

                    for i in 0..loop_pts.len() {
                        let a = loop_pts[i];
                        let b = loop_pts[(i + 1) % loop_pts.len()];
                        if point_on_segment(p, a, b, eps) {
                            return true;
                        }
                    }
                }

                false
            })
            .collect::<Vec<_>>();

        FemMesh {
            positions,
            elements,
            is_boundary,
            element_density: Vec::new(),
            node_density: Vec::new(),
        }
    }

    /// Computes element and node densities from body loops and their densities.
    pub fn compute_density<const N: usize>(&mut self, body_loops: &[[Vec2; N]], bodies: &[Body]) {
        self.element_density = self
            .elements
            .iter()
            .map(|&[i0, i1, i2]| {
                let p0 = self.positions[i0];
                let p1 = self.positions[i1];
                let p2 = self.positions[i2];
                let centroid = (p0 + p1 + p2) / 3.0;

                body_loops
                    .iter()
                    .zip(bodies.iter().map(|b| b.density))
                    .map(|(loop_pts, density)| {
                        if point_in_or_on_polygon(centroid, loop_pts) {
                            density
                        } else {
                            0.0
                        }
                    })
                    .sum()
            })
            .collect();

        // Computes node density values by averaging the densities of its connecting elements.
        let mut node_rho = vec![0.0; self.positions.len()];
        let mut node_count = vec![0; self.positions.len()];

        for (elem_id, node_ids) in self.elements.iter().enumerate() {
            for &i in node_ids {
                node_rho[i] += self.element_density[elem_id];
                node_count[i] += 1;
            }
        }

        for (rho, &count) in node_rho.iter_mut().zip(node_count.iter()) {
            if count > 0 {
                *rho /= count as f32;
            }
        }

        self.node_density = node_rho;
    }
}

pub struct LinearSystem {
    k: CsMat<f32>,   // pub k: HashMap<(usize, usize), f32>,
    f: DVector<f32>, // f: Vec<f32>,
}

impl Operator for LinearSystem {
    type Param = DVector<f32>;

    type Output = DVector<f32>;

    fn apply(&self, param: &Self::Param) -> Result<Self::Output, argmin::core::Error> {
        let mut y = DVector::<f32>::zeros(param.len());

        // CSR: outer_iterator() yields rows
        for (row, row_vec) in self.k.outer_iterator().enumerate() {
            let mut acc = 0.0f32;
            for (col, val) in row_vec.iter() {
                acc += val * param[col];
            }
            y[row] = acc;
        }

        Ok(y)
    }
}

impl LinearSystem {
    /// Builds the system of equations `Kx = f` from a `FemMesh` with computed density values
    /// and specified DIrichlet boundary conditions.
    pub fn from_mesh(mesh: &FemMesh, dirichlet_values: &[Option<f32>]) -> Self {
        let n = mesh.positions.len();

        let mut is_fixed = vec![false; n];
        let mut fixed_value = vec![0.0f32; n];
        for (i, val) in dirichlet_values.iter().enumerate() {
            if let Some(v) = val {
                is_fixed[i] = true;
                fixed_value[i] = *v;
            }
        }

        let mut k_trip = TriMat::<f32>::with_capacity((n, n), mesh.elements.len() * 9 + n);
        let mut f = vec![0.0f32; n];

        for (e_id, &[n0, n1, n2]) in mesh.elements.iter().enumerate() {
            let p0 = mesh.positions[n0];
            let p1 = mesh.positions[n1];
            let p2 = mesh.positions[n2];

            let two_a = (p1.x - p0.x) * (p2.y - p0.y) - (p2.x - p0.x) * (p1.y - p0.y);
            let a = 0.5 * two_a.abs();
            if a <= 1.0e-12 {
                continue;
            }

            let b = [p1.y - p2.y, p2.y - p0.y, p0.y - p1.y];
            let c = [p2.x - p1.x, p0.x - p2.x, p1.x - p0.x];

            let mut k_local = [[0.0f32; 3]; 3];
            for i in 0..3 {
                for j in 0..3 {
                    k_local[i][j] = (b[i] * b[j] + c[i] * c[j]) / (4.0 * a);
                }
            }

            let rho_e = mesh.element_density.get(e_id).copied().unwrap_or(0.0);
            let fe_i = rho_e * a / 3.0;

            let nodes = [n0, n1, n2];

            for &gi in &nodes {
                if !is_fixed[gi] {
                    f[gi] += fe_i;
                }
            }

            for (li, &gi) in nodes.iter().enumerate() {
                if is_fixed[gi] {
                    continue;
                }

                for (lj, &gj) in nodes.iter().enumerate() {
                    let kij = k_local[li][lj];
                    if is_fixed[gj] {
                        // Move known Dirichlet contribution to the RHS.
                        f[gi] -= kij * fixed_value[gj];
                    } else {
                        k_trip.add_triplet(gi, gj, kij);
                    }
                }
            }
        }

        // Replace fixed rows with x_i = g_i.
        for i in 0..n {
            if is_fixed[i] {
                k_trip.add_triplet(i, i, 1.0);
                f[i] = fixed_value[i];
            }
        }

        let k = k_trip.to_csr();
        let f = DVector::from_vec(f);

        LinearSystem { k, f }
    }

    pub fn solve(self, max_iters: u64) -> (DVector<f32>, bool) {
        let b = self.f.clone();
        let x0 = DVector::<f32>::zeros(b.len());

        let solver: ConjugateGradient<DVector<f32>, f32> = ConjugateGradient::new(b);
        let res = Executor::new(self, solver)
            .configure(|state| state.param(x0).max_iters(max_iters))
            .run()
            .unwrap();

        let best_x = res.state().get_best_param().unwrap();
        let termination_status = res.state().get_termination_status().terminated();

        (best_x.clone(), termination_status)

        // // Cost function value associated with best parameter vector
        // let best_cost = res.state().get_best_cost();

        // // Check the execution status

        // // Optionally, check why the optimizer terminated (if status is terminated)
        // let termination_reason = res.state().get_termination_reason();

        // // Time needed for optimization
        // let time_needed = res.state().get_time().unwrap();

        // // Total number of iterations needed
        // let num_iterations = res.state().get_iter();

        // // Iteration number where the last best parameter vector was found
        // let num_iterations_best = res.state().get_last_best_iter();

        // // Number of evaluation counts per method (Cost, Gradient)
        // let function_evaluation_counts = res.state().get_func_counts();
    }
}

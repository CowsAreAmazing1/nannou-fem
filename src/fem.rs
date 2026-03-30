use nannou::prelude::*;
use spade::{ConstrainedDelaunayTriangulation, HasPosition, Triangulation};
use std::collections::HashMap;

#[derive(Debug, Clone)]
pub struct FemMesh {
    pub positions: Vec<Vec2>,
    pub elements: Vec<[usize; 3]>,
    pub is_boundary: Vec<bool>,
}

#[derive(Debug, Clone)]
pub struct PoissonSystem {
    pub rows: Vec<Vec<(usize, f32)>>,
    pub rhs: Vec<f32>,
}

#[derive(Debug, Clone)]
pub struct CgSolveResult {
    pub solution: Vec<f32>,
    pub iterations: usize,
    pub residual_l2: f32,
    pub converged: bool,
}

impl PoissonSystem {
    pub fn node_count(&self) -> usize {
        self.rows.len()
    }

    pub fn nnz(&self) -> usize {
        self.rows.iter().map(Vec::len).sum()
    }
}

fn sparse_matvec(system: &PoissonSystem, x: &[f32]) -> Vec<f32> {
    system
        .rows
        .iter()
        .map(|row| row.iter().map(|(j, a)| *a * x[*j]).sum::<f32>())
        .collect::<Vec<_>>()
}

fn dot(a: &[f32], b: &[f32]) -> f32 {
    a.iter().zip(b.iter()).map(|(x, y)| x * y).sum::<f32>()
}

pub fn apply_dirichlet_boundary<G>(system: &mut PoissonSystem, mesh: &FemMesh, boundary_value: G)
where
    G: Fn(usize, Vec2) -> f32,
{
    let mut prescribed = vec![None; system.rows.len()];

    for (i, &is_boundary) in mesh.is_boundary.iter().enumerate() {
        if is_boundary {
            prescribed[i] = Some(boundary_value(i, mesh.positions[i]));
        }
    }

    for (i, bc) in prescribed.iter().enumerate() {
        if bc.is_some() {
            continue;
        }

        let mut filtered = Vec::with_capacity(system.rows[i].len());
        for &(j, aij) in &system.rows[i] {
            if let Some(phi_j) = prescribed[j] {
                system.rhs[i] -= aij * phi_j;
            } else {
                filtered.push((j, aij));
            }
        }
        system.rows[i] = filtered;
    }

    for (i, bc) in prescribed.iter().enumerate() {
        if let Some(phi_i) = *bc {
            system.rows[i].clear();
            system.rows[i].push((i, 1.0));
            system.rhs[i] = phi_i;
        }
    }
}

pub fn conjugate_gradient_solve(
    system: &PoissonSystem,
    tolerance: f32,
    max_iterations: usize,
) -> CgSolveResult {
    let n = system.node_count();
    if n == 0 {
        return CgSolveResult {
            solution: Vec::new(),
            iterations: 0,
            residual_l2: 0.0,
            converged: true,
        };
    }

    let mut x = vec![0.0_f32; n];
    let mut r = system.rhs.clone();
    let mut p = r.clone();
    let mut rs_old = dot(&r, &r);

    if rs_old.sqrt() <= tolerance {
        return CgSolveResult {
            solution: x,
            iterations: 0,
            residual_l2: rs_old.sqrt(),
            converged: true,
        };
    }

    for k in 0..max_iterations {
        let ap = sparse_matvec(system, &p);
        let denom = dot(&p, &ap);
        if denom.abs() <= f32::EPSILON {
            return CgSolveResult {
                solution: x,
                iterations: k,
                residual_l2: rs_old.sqrt(),
                converged: false,
            };
        }

        let alpha = rs_old / denom;

        for i in 0..n {
            x[i] += alpha * p[i];
            r[i] -= alpha * ap[i];
        }

        let rs_new = dot(&r, &r);
        let residual = rs_new.sqrt();
        if residual <= tolerance {
            return CgSolveResult {
                solution: x,
                iterations: k + 1,
                residual_l2: residual,
                converged: true,
            };
        }

        let beta = rs_new / rs_old;
        for i in 0..n {
            p[i] = r[i] + beta * p[i];
        }
        rs_old = rs_new;
    }

    CgSolveResult {
        solution: x,
        iterations: max_iterations,
        residual_l2: rs_old.sqrt(),
        converged: false,
    }
}

fn point_on_segment(p: Vec2, a: Vec2, b: Vec2, eps: f32) -> bool {
    let ab = b - a;
    let ap = p - a;

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

pub fn build_binary_domain_values<V>(
    triangulation: &ConstrainedDelaunayTriangulation<V>,
    constrained_loop: &[Vec2],
) -> Vec<f32>
where
    V: HasPosition<Scalar = f32>,
{
    triangulation
        .vertices()
        .map(|vert| {
            let p = vert.position();
            if point_in_or_on_polygon(vec2(p.x, p.y), constrained_loop) {
                1.0
            } else {
                0.0
            }
        })
        .collect()
}

pub fn build_mesh<V>(
    triangulation: &ConstrainedDelaunayTriangulation<V>,
    constrained_loop: &[Vec2],
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

    let is_boundary = positions
        .iter()
        .map(|&p| {
            if constrained_loop.len() < 2 {
                return false;
            }

            let max_extent = constrained_loop
                .iter()
                .map(|v| v.length())
                .fold(1.0_f32, f32::max);
            let eps = 1.0e-4 * max_extent;

            for i in 0..constrained_loop.len() {
                let a = constrained_loop[i];
                let b = constrained_loop[(i + 1) % constrained_loop.len()];
                if point_on_segment(p, a, b, eps) {
                    return true;
                }
            }
            false
        })
        .collect::<Vec<_>>();

    FemMesh {
        positions,
        elements,
        is_boundary,
    }
}

pub fn assemble_poisson_system<F>(mesh: &FemMesh, source_density: F) -> PoissonSystem
where
    F: Fn(Vec2) -> f32,
{
    let n = mesh.positions.len();
    let mut rows_map = vec![HashMap::<usize, f32>::new(); n];
    let mut rhs = vec![0.0_f32; n];

    for &[i0, i1, i2] in &mesh.elements {
        let p0 = mesh.positions[i0];
        let p1 = mesh.positions[i1];
        let p2 = mesh.positions[i2];

        let x0 = p0.x;
        let y0 = p0.y;
        let x1 = p1.x;
        let y1 = p1.y;
        let x2 = p2.x;
        let y2 = p2.y;

        let two_a = (x1 - x0) * (y2 - y0) - (x2 - x0) * (y1 - y0);
        let area = 0.5 * two_a.abs();
        if area <= f32::EPSILON {
            continue;
        }

        let b = [y1 - y2, y2 - y0, y0 - y1];
        let c = [x2 - x1, x0 - x2, x1 - x0];

        let ids = [i0, i1, i2];

        for a in 0..3 {
            for d in 0..3 {
                let ke = (b[a] * b[d] + c[a] * c[d]) / (4.0 * area);
                *rows_map[ids[a]].entry(ids[d]).or_insert(0.0) += ke;
            }
        }

        let centroid = (p0 + p1 + p2) / 3.0;
        let rho = source_density(centroid);
        let fe = rho * area / 3.0;
        rhs[i0] += fe;
        rhs[i1] += fe;
        rhs[i2] += fe;
    }

    let rows = rows_map
        .into_iter()
        .map(|row_map| {
            let mut row = row_map.into_iter().collect::<Vec<_>>();
            row.sort_by_key(|(j, _)| *j);
            row
        })
        .collect::<Vec<_>>();

    PoissonSystem { rows, rhs }
}

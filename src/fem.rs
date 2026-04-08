use nannou::prelude::*;
use spade::{ConstrainedDelaunayTriangulation, HasPosition, Triangulation};

#[derive(Debug, Clone)]
pub struct FemMesh {
    pub positions: Vec<Vec2>,
    pub elements: Vec<[usize; 3]>,
    pub is_boundary: Vec<bool>,
}

// fn dot(a: &[f32], b: &[f32]) -> f32 {
//     a.iter().zip(b.iter()).map(|(x, y)| x * y).sum::<f32>()
// }

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

pub fn point_in_any_polygon(point: Vec2, polygons: &[Vec<Vec2>]) -> bool {
    for polygon in polygons {
        if point_in_or_on_polygon(point, polygon) {
            return true;
        }
    }
    false
}

/// Converts a `ConstrainedDelaunayTriangulation` to a `FemMesh`, an easy-to-use format for finite element assembly and solving. The `is_boundary` field is determined by checking if each vertex lies on any of the constrained loops.
pub fn build_mesh<V>(
    triangulation: &ConstrainedDelaunayTriangulation<V>,
    constrained_loops: &[Vec<Vec2>],
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
    }
}

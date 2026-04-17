use nannou::prelude::*;

pub struct PortalPair {
    portal1: Portal,
    portal2: Portal,
}

impl PortalPair {
    fn from_positions(pos1a: Vec2, pos1b: Vec2, pos2a: Vec2, pos2b: Vec2) -> Self {
        Self {
            portal1: Portal {
                pos1: pos1a,
                pos2: pos1b,
            },
            portal2: Portal {
                pos1: pos2a,
                pos2: pos2b,
            },
        }
    }

    fn from_portals(portal1: Portal, portal2: Portal) -> Self {
        Self { portal1, portal2 }
    }
}

impl PortalPair {}

pub struct Portal {
    pos1: Vec2,
    pos2: Vec2,
}

impl Portal {
    fn new(pos1: Vec2, pos2: Vec2) -> Self {
        Self { pos1, pos2 }
    }
}

/// A body is a closed parametric shape with a density, contributing mass to the system.
pub struct Body {
    generator: Box<dyn Fn(f32) -> Vec2>,
    pub(crate) density: f32,
    resolution: usize,
}

impl Body {
    // Builds a body from a parametric curve `generator` function that maps [0, 1] to `resolution` points on the boundary, and the shape's `density`
    pub fn new(density: f32, resolution: usize, generator: impl Fn(f32) -> Vec2 + 'static) -> Self {
        Self {
            generator: Box::new(generator),
            density,
            resolution,
        }
    }

    pub fn sample_boundary(&self) -> Vec<Vec2> {
        (0..self.resolution)
            .map(|i| {
                let t = i as f32 / self.resolution as f32;
                (self.generator)(t)
            })
            .collect()
    }
}

use nannou::prelude::*;

pub trait Sampleable {
    const CLOSED: bool;

    fn sample(&self) -> Vec<Vec<Vec2>>;
    fn density(&self) -> Option<f32> {
        None
    }
}

#[derive(Clone)]
pub struct Portal {
    pos1: Vec2,
    pos2: Vec2,
}

impl Portal {
    fn new(pos1: Vec2, pos2: Vec2) -> Self {
        Self { pos1, pos2 }
    }
}

impl Sampleable for Portal {
    const CLOSED: bool = false;

    fn sample(&self) -> Vec<Vec<Vec2>> {
        vec![vec![self.pos1, self.pos2]]
    }
}

#[derive(Clone)]
pub struct PortalPair {
    portal1: Portal,
    portal2: Portal,
}

impl PortalPair {
    pub fn from_positions(pos1a: Vec2, pos1b: Vec2, pos2a: Vec2, pos2b: Vec2) -> Self {
        Self {
            portal1: Portal::new(pos1a, pos1b),
            portal2: Portal::new(pos2a, pos2b),
        }
    }

    fn _from_portals(portal1: Portal, portal2: Portal) -> Self {
        Self { portal1, portal2 }
    }
}

// TREAT EACH PORTAL AS A SINGLE OBJECT AS THE END OF PORTAL 1 DOES NOT CONNECT TO THE START OF PORTAL 2
impl Sampleable for PortalPair {
    const CLOSED: bool = false;

    fn sample(&self) -> Vec<Vec<Vec2>> {
        let mut points = Vec::new();
        points.extend(self.portal1.sample());
        points.extend(self.portal2.sample());
        points
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

impl Sampleable for Body {
    const CLOSED: bool = true;

    fn sample(&self) -> Vec<Vec<Vec2>> {
        vec![self.sample_boundary()]
    }

    fn density(&self) -> Option<f32> {
        Some(self.density)
    }
}

#[derive(Default)]
pub struct ConstraintHandler {
    pub loops: Vec<Vec<Vec2>>,
    closed: Vec<bool>,
    density: Vec<Option<f32>>,
}

impl ConstraintHandler {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn add_object<O>(mut self, object: O) -> Self
    where
        O: Sampleable,
    {
        for loop_points in object.sample() {
            self.loops.push(loop_points);
            self.closed.push(O::CLOSED);
            self.density.push(object.density());
        }

        self
    }

    pub fn get_constraints(&self) -> Vec<(Vec<Vec2>, bool)> {
        // (self.loops.clone(), self.closed.clone())
        self.loops
            .iter()
            .cloned()
            .zip(self.closed.iter().cloned())
            .collect()
    }

    pub fn get_dense(&self) -> Vec<(Vec<Vec2>, f32)> {
        self.loops
            .iter()
            .zip(self.density.clone())
            .filter_map(|(loop_points, density)| density.map(|d| (loop_points.clone(), d)))
            .collect()
    }
}

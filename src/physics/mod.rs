// Exports submodules

pub mod bodies;
// pub mod integrator; // Removed
pub mod collisions;

use bodies::{Body, BodyType};
use collisions::{detect_collisions, resolve_collisions};

/// Represents the physics world that contains all bodies and handles simulation
pub struct World {
    /// Collection of all physical bodies in the simulation
    pub bodies: Vec<Body>,
    /// Gravity vector
    pub gravity: nalgebra::Vector2<f64>,
}

impl World {
    /// Creates a new physics world
    pub fn new() -> Self {
        Self {
            bodies: Vec::new(),
            gravity: nalgebra::Vector2::new(0.0, -9.81), // Default gravity pointing down
        }
    }

    /// Adds a body to the world
    pub fn add_body(&mut self, body: Body) {
        self.bodies.push(body);
    }

    /// Updates the physics simulation by one time step
    pub fn update(&mut self, dt: f64) {
        // 1. Reset forces for all bodies
        for body in &mut self.bodies {
            body.force = nalgebra::Vector2::zeros();
            // Keep acceleration from previous step until recalculated in body.update
        }

        // 2. Apply global forces (like gravity)
        for body in &mut self.bodies {
            if let BodyType::Dynamic = body.body_type {
                body.apply_force(self.gravity * body.mass);
            }
        }

        // 3. Update body positions/velocities (integrates forces -> acceleration -> velocity -> position)
        for body in &mut self.bodies {
            body.update(dt);
        }

        // 4. Iteratively resolve collisions (applies impulse-based velocity changes)
        const SOLVER_ITERATIONS: u32 = 10;
        for _ in 0..SOLVER_ITERATIONS {
            let collisions = detect_collisions(&self.bodies);
            if collisions.is_empty() {
                break;
            }
            resolve_collisions(self, &collisions);
        }
    }
}

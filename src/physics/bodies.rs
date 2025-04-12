/// Definition of physical bodies

use nalgebra::{Point2, Vector2};

/// Different types of shapes a body can have
#[derive(Debug, Clone)]
pub enum Shape {
    Circle { radius: f64 },
    Rectangle { width: f64, height: f64 },
}

/// Material properties for different types of objects
#[derive(Debug, Clone)]
pub struct Material {
    /// Density of the material (kg/m^2)
    pub density: f64,
    /// Coefficient of restitution (bounciness)
    pub restitution: f64,
    /// Coefficient of friction
    pub friction: f64,
}

impl Material {
    pub fn new(density: f64, restitution: f64, friction: f64) -> Self {
        Self {
            density,
            restitution,
            friction,
        }
    }

    /// Predefined materials
    pub fn wood() -> Self {
        Self::new(0.5, 0.3, 0.3)
    }

    pub fn stone() -> Self {
        Self::new(2.0, 0.1, 0.7)
    }

    pub fn rubber() -> Self {
        Self::new(0.3, 0.8, 0.7)
    }
}

/// Different types of bodies
#[derive(Debug, Clone, PartialEq)]
pub enum BodyType {
    Static,  // Immovable bodies (ground, walls)
    Dynamic, // Normal physics bodies
}

/// Represents a physical body in the simulation
#[derive(Debug, Clone)]
// Allow dead code since features like rotation are planned but not implemented
#[allow(dead_code)] 
pub struct Body {
    /// Position of the body's center of mass
    pub position: Point2<f64>,
    /// Linear velocity of the body
    pub velocity: Vector2<f64>,
    /// Linear acceleration of the body
    pub acceleration: Vector2<f64>,
    /// Mass of the body in kilograms
    pub mass: f64,
    /// Shape of the body
    pub shape: Shape,
    /// Material properties
    pub material: Material,
    /// Type of body (static or dynamic)
    pub body_type: BodyType,
    /// Force applied to the body
    pub force: Vector2<f64>,
}

impl Body {
    /// Creates a new body with the given properties
    pub fn new(
        position: Point2<f64>,
        shape: Shape,
        material: Material,
        body_type: BodyType,
    ) -> Self {
        let mass = match &shape {
            Shape::Circle { radius } => std::f64::consts::PI * radius * radius * material.density,
            Shape::Rectangle { width, height } => width * height * material.density,
        };

        Self {
            position,
            velocity: Vector2::new(0.0, 0.0),
            acceleration: Vector2::new(0.0, 0.0),
            mass,
            shape,
            material,
            body_type,
            force: Vector2::zeros(),
        }
    }

    /// Creates a new circle body
    pub fn new_circle(
        position: Point2<f64>,
        radius: f64,
        material: Material,
        body_type: BodyType,
    ) -> Self {
        Self::new(
            position,
            Shape::Circle { radius },
            material,
            body_type,
        )
    }

    /// Creates a new rectangle body
    pub fn new_rectangle(
        position: Point2<f64>,
        width: f64,
        height: f64,
        material: Material,
        body_type: BodyType,
    ) -> Self {
        Self::new(
            position,
            Shape::Rectangle { width, height },
            material,
            body_type,
        )
    }

    /// Applies a force to the body, accumulating it for the next update step.
    pub fn apply_force(&mut self, force_to_apply: Vector2<f64>) {
        if let BodyType::Dynamic = self.body_type {
            self.force += force_to_apply;
        }
    }

    /// Updates the body's state using semi-implicit Euler integration
    pub fn update(&mut self, dt: f64) {
        if self.body_type == BodyType::Static {
            return;
        }

        // Calculate acceleration from accumulated forces (F=ma => a=F/m)
        self.acceleration = self.force / self.mass;

        // Update velocity based on acceleration
        self.velocity += self.acceleration * dt;
        
        // Update position based on the new velocity
        self.position += self.velocity * dt;
        
        // NOTE: Forces are now reset in World::update *before* gravity is applied
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_material_properties() {
        let wood = Material::wood();
        assert_eq!(wood.density, 0.5);
        assert_eq!(wood.restitution, 0.3);
        assert_eq!(wood.friction, 0.3);

        let stone = Material::stone();
        assert_eq!(stone.density, 2.0);
        assert_eq!(stone.restitution, 0.1);
        assert_eq!(stone.friction, 0.7);

        let rubber = Material::rubber();
        assert_eq!(rubber.density, 0.3);
        assert_eq!(rubber.restitution, 0.8);
        assert_eq!(rubber.friction, 0.7);
    }

    #[test]
    fn test_circle_body_creation() {
        let position = Point2::new(10.0, 20.0);
        let radius = 5.0;
        let material = Material::wood();
        let body_type = BodyType::Dynamic;

        let circle = Body::new_circle(position, radius, material.clone(), body_type);

        assert_eq!(circle.position, position);
        assert_eq!(circle.velocity, Vector2::new(0.0, 0.0));
        assert_eq!(circle.acceleration, Vector2::new(0.0, 0.0));
        assert!((circle.mass - std::f64::consts::PI * radius * radius * material.density).abs() < 1e-10);
    }

    #[test]
    fn test_rectangle_body_creation() {
        let position = Point2::new(10.0, 20.0);
        let width = 10.0;
        let height = 20.0;
        let material = Material::stone();
        let body_type = BodyType::Static;

        let rect = Body::new_rectangle(position, width, height, material.clone(), body_type);

        assert_eq!(rect.position, position);
        assert_eq!(rect.velocity, Vector2::new(0.0, 0.0));
        assert_eq!(rect.acceleration, Vector2::new(0.0, 0.0));
        assert_eq!(rect.mass, width * height * material.density);
    }

    #[test]
    fn test_force_application() {
        let mut body = Body::new_circle(
            Point2::new(0.0, 0.0),
            1.0,
            Material::wood(),
            BodyType::Dynamic,
        );

        let force = Vector2::new(10.0, -5.0);
        body.apply_force(force);

        // Check that the force was accumulated correctly
        assert!((body.force - force).norm() < 1e-10, "Force not accumulated correctly");
        // Acceleration shouldn't change until update() is called
        assert_eq!(body.acceleration, Vector2::new(0.0, 0.0), "Acceleration changed prematurely");
    }

    #[test]
    fn test_static_body_force() {
        let mut body = Body::new_circle(
            Point2::new(0.0, 0.0),
            1.0,
            Material::stone(),
            BodyType::Static,
        );

        let initial_pos = body.position;
        let initial_vel = body.velocity;
        let initial_acc = body.acceleration;

        body.apply_force(Vector2::new(10.0, -5.0));
        body.update(1.0);

        // Static bodies should not move
        assert_eq!(body.position, initial_pos);
        assert_eq!(body.velocity, initial_vel);
        assert_eq!(body.acceleration, initial_acc);
    }

    #[test]
    fn test_dynamic_body_update() {
        let mut body = Body::new_circle(
            Point2::new(0.0, 0.0),
            1.0,
            Material::wood(),
            BodyType::Dynamic,
        );

        body.velocity = Vector2::new(2.0, 1.0);
        body.acceleration = Vector2::new(-1.0, 0.5);
        let dt = 0.1;

        // Calculate expected state AFTER update
        let expected_velocity = body.velocity + body.acceleration * dt;
        // Use the NEW expected velocity to calculate the expected position
        let expected_position = body.position + expected_velocity * dt;

        body.update(dt);

        assert!((body.velocity - expected_velocity).norm() < 1e-10, "Velocity mismatch");
        assert!((body.position - expected_position).norm() < 1e-10, "Position mismatch");
        assert_eq!(body.acceleration, Vector2::new(0.0, 0.0), "Acceleration not reset");
    }
}


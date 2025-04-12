/// Collision detection and resolution

use nalgebra::{/* Point2, */ Vector2};
use crate::physics::bodies::{Body, Shape, BodyType};

/// Represents a collision between two bodies
pub struct Collision {
    /// The first body involved in the collision
    pub body_a: usize,
    /// The second body involved in the collision
    pub body_b: usize,
    /// The normal vector of the collision (pointing from body_a to body_b)
    pub normal: Vector2<f64>,
}

/// Detects collisions between bodies and returns a list of collisions
pub fn detect_collisions(bodies: &[Body]) -> Vec<Collision> {
    let mut collisions = Vec::new();
    
    for i in 0..bodies.len() {
        for j in (i + 1)..bodies.len() {
            let body_a = &bodies[i];
            let body_b = &bodies[j];
            
            // Skip collision if both bodies are static
            if let (BodyType::Static, BodyType::Static) = (&body_a.body_type, &body_b.body_type) {
                continue;
            }

            if let Some(collision) = check_collision(body_a, body_b, i, j) {
                collisions.push(collision);
            }
        }
    }
    
    collisions
}

/// Checks for collision between two bodies
fn check_collision(body_a: &Body, body_b: &Body, index_a: usize, index_b: usize) -> Option<Collision> {
    // Ensure index_a is always smaller than index_b for consistent ordering
    let (body1, body2, idx1, idx2) = if index_a < index_b {
        (body_a, body_b, index_a, index_b)
    } else {
        (body_b, body_a, index_b, index_a)
    };

    let collision_result = match (&body1.shape, &body2.shape) {
        (Shape::Circle { radius: r1 }, Shape::Circle { radius: r2 }) => {
            // Circle-Circle collision
            let diff = body2.position - body1.position;
            let distance = diff.norm();
            let min_distance = r1 + r2;
            
            if distance < min_distance && distance > 1e-10 {
                let normal = diff / distance;
                
                Some(Collision {
                    body_a: idx1,
                    body_b: idx2,
                    normal, // Points from 1 to 2
                })
            } else {
                None
            }
        }
        (Shape::Rectangle { width: w1, height: h1 }, Shape::Rectangle { width: w2, height: h2 }) => {
            // Rectangle-Rectangle collision using AABB
            let half_size1 = Vector2::new(w1 / 2.0, h1 / 2.0);
            let half_size2 = Vector2::new(w2 / 2.0, h2 / 2.0);
            
            let diff = body2.position - body1.position;
            let abs_diff = Vector2::new(diff.x.abs(), diff.y.abs());
            
            let overlap = half_size1 + half_size2 - abs_diff;
            
            if overlap.x > 0.0 && overlap.y > 0.0 {
                let normal = if overlap.x < overlap.y {
                    Vector2::new(diff.x.signum(), 0.0)
                } else {
                    Vector2::new(0.0, diff.y.signum())
                };
                
                Some(Collision {
                    body_a: idx1,
                    body_b: idx2,
                    normal, // Points from 1 to 2
                })
            } else {
                None
            }
        }
        (Shape::Circle { radius }, Shape::Rectangle { width, height }) => {
            // Circle (body1) vs Rectangle (body2)
            calculate_circle_rectangle_collision(body1, body2, idx1, idx2, *radius, *width, *height)
        }
        (Shape::Rectangle { width, height }, Shape::Circle { radius }) => {
             // Rectangle (body1) vs Circle (body2)
            // Calculate as Circle-Rect
            calculate_circle_rectangle_collision(body2, body1, idx2, idx1, *radius, *width, *height)
                .map(|mut c| {
                    // Swap bodies back to original order (idx1 < idx2)
                    c.body_a = idx1;
                    c.body_b = idx2;
                    // The normal from calculate_circle_rectangle_collision already points from rect (1) to circle (2).
                    // DO NOT flip it.
                    c
                })
        }
    };

    collision_result
}

// Helper function for Circle-Rectangle collision
fn calculate_circle_rectangle_collision(
    circle_body: &Body, 
    rect_body: &Body, 
    circle_idx: usize, 
    rect_idx: usize, 
    radius: f64, 
    width: f64, 
    height: f64
) -> Option<Collision> {
    let circle_center = circle_body.position;
    let rect_center = rect_body.position;
    let half_extents = Vector2::new(width / 2.0, height / 2.0);

    let delta = circle_center - rect_center;
    let clamped_x = delta.x.clamp(-half_extents.x, half_extents.x);
    let clamped_y = delta.y.clamp(-half_extents.y, half_extents.y);
    let closest_point = rect_center + Vector2::new(clamped_x, clamped_y);

    let collision_vector = circle_center - closest_point;
    let distance_sq = collision_vector.norm_squared();
    let radius_sq = radius * radius;

    if distance_sq < radius_sq && distance_sq > 1e-12 {
        let distance = distance_sq.sqrt();
        let normal = collision_vector / distance; // Normal points from rect towards circle

        Some(Collision {
            body_a: circle_idx, // Circle index
            body_b: rect_idx,   // Rectangle index
            normal,
        })
    } else {
        None
    }
}

/// Resolves collisions by applying impulses to the bodies
pub fn resolve_collisions(world: &mut crate::physics::World, collisions: &[Collision]) {
    for collision in collisions {
        let (first, second) = world.bodies.split_at_mut(collision.body_a + 1);
        let body_a = &mut first[collision.body_a];
        let body_b = &mut second[collision.body_b - collision.body_a - 1];
        
        // Skip if both bodies are static
        if let (BodyType::Static, BodyType::Static) = (&body_a.body_type, &body_b.body_type) {
            continue;
        }
        
        // Calculate relative velocity
        let relative_velocity = body_b.velocity - body_a.velocity;
        
        // Calculate relative velocity along the normal
        let velocity_along_normal = relative_velocity.dot(&collision.normal);
        
        // If bodies are moving apart, skip resolution
        if velocity_along_normal > 0.0 {
            continue;
        }
        
        // Calculate restitution (bounciness)
        let restitution = (body_a.material.restitution + body_b.material.restitution) / 2.0;
        
        // Calculate inverse masses (0 for static bodies)
        let inv_mass_a = if body_a.body_type == BodyType::Static { 0.0 } else { 1.0 / body_a.mass };
        let inv_mass_b = if body_b.body_type == BodyType::Static { 0.0 } else { 1.0 / body_b.mass };
        let total_inv_mass = inv_mass_a + inv_mass_b;

        // Ensure we don't divide by zero (shouldn't happen due to the static-static check earlier)
        if total_inv_mass == 0.0 {
            continue;
        }
        
        // Calculate impulse scalar
        let j = -(1.0 + restitution) * velocity_along_normal;
        let impulse_scalar = j / total_inv_mass;
        
        // Apply impulse using inverse masses
        let impulse = collision.normal * impulse_scalar;
        if body_a.body_type == BodyType::Dynamic {
            body_a.velocity -= impulse * inv_mass_a;
        }
        if body_b.body_type == BodyType::Dynamic {
            body_b.velocity += impulse * inv_mass_b;
        }

        // Friction Impulse Calculation
        let friction_tolerance = 1e-7;

        // Recalculate relative velocity AFTER normal impulse is applied
        let relative_velocity_friction = body_b.velocity - body_a.velocity; 

        // Project relative velocity onto the normal vector
        let velocity_normal_comp = collision.normal * relative_velocity_friction.dot(&collision.normal);
        // Calculate the tangential component of the relative velocity
        let velocity_tangent_comp = relative_velocity_friction - velocity_normal_comp;
        let tangential_speed = velocity_tangent_comp.norm();

        // Apply friction impulse if tangential speed is significant
        if tangential_speed > friction_tolerance {
            // Direction of friction opposes tangential relative motion
            let tangent_direction = velocity_tangent_comp / tangential_speed;

            // Calculate impulse magnitude needed to stop tangential motion
            let jt = -tangential_speed / total_inv_mass;

            // Calculate static friction limit
            let mu_static = (body_a.material.friction + body_b.material.friction) / 2.0;
            let max_friction_impulse = mu_static * impulse_scalar.abs();

            // Clamp friction impulse magnitude by the static friction limit
            let friction_impulse_scalar = jt.clamp(-max_friction_impulse, max_friction_impulse);

            // Calculate final friction impulse vector
            let friction_impulse = tangent_direction * friction_impulse_scalar;

            // Apply friction impulse
            if body_a.body_type == BodyType::Dynamic {
                body_a.velocity -= friction_impulse * inv_mass_a;
            }
            if body_b.body_type == BodyType::Dynamic {
                body_b.velocity += friction_impulse * inv_mass_b;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::physics::World;

    #[test]
    fn test_circle_circle_collision() {
        let body_a = Body::new_circle(
            Point2::new(0.0, 0.0),
            2.0,
            Material::wood(),
            BodyType::Dynamic,
        );
        let body_b = Body::new_circle(
            Point2::new(3.0, 0.0),
            2.0,
            Material::wood(),
            BodyType::Dynamic,
        );

        let collision = check_collision(&body_a, &body_b, 0, 1);
        assert!(collision.is_some());

        let collision = collision.unwrap();
        assert_eq!(collision.body_a, 0);
        assert_eq!(collision.body_b, 1);
        assert!((collision.normal - Vector2::new(1.0, 0.0)).norm() < 1e-10);
        assert!((collision.depth - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_circle_circle_no_collision() {
        let body_a = Body::new_circle(
            Point2::new(0.0, 0.0),
            2.0,
            Material::wood(),
            BodyType::Dynamic,
        );
        let body_b = Body::new_circle(
            Point2::new(5.0, 0.0),
            2.0,
            Material::wood(),
            BodyType::Dynamic,
        );

        let collision = check_collision(&body_a, &body_b, 0, 1);
        assert!(collision.is_none());
    }

    #[test]
    fn test_rectangle_rectangle_collision() {
        let body_a = Body::new_rectangle(
            Point2::new(0.0, 0.0),
            4.0,
            4.0,
            Material::wood(),
            BodyType::Dynamic,
        );
        let body_b = Body::new_rectangle(
            Point2::new(3.0, 0.0),
            4.0,
            4.0,
            Material::wood(),
            BodyType::Dynamic,
        );

        let collision = check_collision(&body_a, &body_b, 0, 1);
        assert!(collision.is_some());

        let collision = collision.unwrap();
        assert_eq!(collision.body_a, 0);
        assert_eq!(collision.body_b, 1);
        assert!((collision.normal - Vector2::new(1.0, 0.0)).norm() < 1e-10);
        assert!((collision.depth - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_static_bodies_no_collision_check() {
        let mut world = World::new();
        
        let body_a = Body::new_circle(
            Point2::new(0.0, 0.0),
            2.0,
            Material::stone(),
            BodyType::Static,
        );
        let body_b = Body::new_circle(
            Point2::new(3.0, 0.0),
            2.0,
            Material::stone(),
            BodyType::Static,
        );

        world.add_body(body_a);
        world.add_body(body_b);

        let collisions = detect_collisions(&world.bodies);
        assert!(collisions.is_empty());
    }

    #[test]
    fn test_collision_resolution() {
        let mut world = World::new();
        
        // Use a perfectly elastic material for this test
        let perfectly_elastic_material = Material::new(0.3, 1.0, 0.7); // density=0.3, e=1.0, friction=0.7

        let mut body_a = Body::new_circle(
            Point2::new(0.0, 0.0),
            2.0,
            perfectly_elastic_material.clone(),
            BodyType::Dynamic,
        );
        let mut body_b = Body::new_circle(
            Point2::new(3.0, 0.0),
            2.0,
            perfectly_elastic_material.clone(),
            BodyType::Dynamic,
        );

        // Set initial velocities
        body_a.velocity = Vector2::new(1.0, 0.0);
        body_b.velocity = Vector2::new(-1.0, 0.0);

        world.add_body(body_a);
        world.add_body(body_b);

        // Detect and resolve collisions
        let collisions = detect_collisions(&world.bodies);
        resolve_collisions(&mut world, &collisions);

        // After perfectly elastic collision, velocities should be swapped
        assert!((world.bodies[0].velocity - Vector2::new(-1.0, 0.0)).norm() < 1e-10);
        assert!((world.bodies[1].velocity - Vector2::new(1.0, 0.0)).norm() < 1e-10);
    }

    #[test]
    fn test_static_dynamic_collision() {
        let mut world = World::new();
        
        let static_body = Body::new_rectangle(
            Point2::new(0.0, 0.0),
            4.0,
            4.0,
            Material::stone(),
            BodyType::Static,
        );
        let mut dynamic_body = Body::new_circle(
            Point2::new(3.0, 0.0),
            2.0,
            Material::rubber(),
            BodyType::Dynamic,
        );

        // Set initial velocity towards static body
        dynamic_body.velocity = Vector2::new(-1.0, 0.0);

        world.add_body(static_body);
        world.add_body(dynamic_body);

        // Detect and resolve collisions
        let collisions = detect_collisions(&world.bodies);
        resolve_collisions(&mut world, &collisions);

        // Static body should not move
        assert_eq!(world.bodies[0].position, Point2::new(0.0, 0.0));
        assert_eq!(world.bodies[0].velocity, Vector2::new(0.0, 0.0));

        // Dynamic body should bounce back
        assert!(world.bodies[1].velocity.x > 0.0);
    }
}



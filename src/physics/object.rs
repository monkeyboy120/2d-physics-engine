use std::ops::{Add, Sub, Mul};

#[derive(Clone, Copy, Debug)]

struct Vec2 {
    x: f64,
    y: f64,
}

impl Vec2 {
   fn new(x: f64, y: f64) -> Vec2 {
       Vec2 { x, y }
   }

    fn magnitude(&self) -> f64 {
        (self.x * self.x + self.y * self.y).sqrt()
    }
}

impl Add for Vec2 {
    type Output = Vec2;

    fn add(self, rhs: Vec2) -> Vec2 {
        Vec2 {
            x: self.x + rhs.x,
            y: self.y + rhs.y,
        }
    }
}

impl Sub for Vec2 {
    type Output = Vec2;

    fn sub(self, rhs: Vec2) -> Vec2 {
        Vec2 {
            x: self.x - rhs.x,
            y: self.y - rhs.y,
        }
    }
}

impl Mul<f64> for Vec2 {
    type Output = Vec2;

    fn mul(self, rhs: f64) -> Vec2 {
        Vec2 {
            x: self.x * rhs,
            y: self.y * rhs,
        }
    }
}

impl Vec2 {
    fn dot(&self, other: Vec2) -> f64 {
        self.x * other.x + self.y * other.y
    }
}

enum Shape {
    Rectangle(Rectangle),
    Circle(Circle),
}
struct Rectangle {
    min: Vec2,
    max: Vec2,
} 

struct Circle {
center: Vec2,
    radius: f64,
}

impl Circle {
    fn collides_with_circle(&self, other: Circle) -> bool {
        let distance = (self.center - other.center).magnitude();
        distance < self.radius + other.radius
    }
}

impl Rectangle {
    fn collides_with_rectangle(&self, other: Rectangle) -> bool {
        if self.max.x < other.min.x || self.min.x > other.max.x {
            return false
        }
        if self.max.y < other.min.y || self.min.y > other.max.y {
            return false
        }
        return true
    }
}

fn circle_rectangle_collision(circle: Circle, rectangle: Rectangle) -> bool {
    let mut closest_x = circle.center.x;
    let mut closest_y = circle.center.y;

    if circle.center.x < rectangle.min.x {
        closest_x = rectangle.min.x;
    } else if circle.center.x > rectangle.max.x {
        closest_x = rectangle.max.x;
    }

    if circle.center.y < rectangle.min.y {
        closest_y = rectangle.min.y;
    } else if circle.center.y > rectangle.max.y {
        closest_y = rectangle.max.y;
    }

    let distance = Vec2::new(circle.center.x - closest_x, circle.center.y - closest_y);
    distance.magnitude() < circle.radius
}
struct Object {
    position: Vec2,
    velocity: Vec2,
    acceleration: Vec2,
    mass: f64,
    shape: Shape,
}

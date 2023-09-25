struct Vec2 {
    x: f64,
    y: f64,
}

impl Vec2 {
   fn new(x: f64, y: f64) -> Vec2 {
       Vec2 { x, y }
   }

    fn add(&self, other: &Vec2) -> Vec2 {
        Vec2::new(self.x + other.x, self.y + other.y)
    }

    fn sub(&self, other: &Vec2) -> Vec2 {
        Vec2::new(self.x - other.x, self.y - other.y)
    }

    fn scale(&self, scalar: f64) -> Vec2 {
        Vec2::new(self.x * scalar, self.y * scalar)
    }

    fn dot(&self, other: &Vec2) -> f64 {
        self.x * other.x + self.y * other.y
    }

    fn magnitude(&self) -> f64 {
        (self.x * self.x + self.y * self.y).sqrt()
    }
}

struct Rectangle {
    min: Vec2,
    max: Vec2,
} 

struct Circle {
center: Vec2,
    radius: f64,
}
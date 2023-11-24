fn rectangle_collision(a: Rectangle, b: Rectangle) -> bool {
    if a.max.x < b.min.x || a.min.x > b.max.x {
        return false
    }
    if a.max.y < b.min.y || a.min.y > b.max.y { return false }

    return true
}

fn distance( a: Vec2, b: Vec2 ) -> f64 {
    return ((a.x-b.x) * (a.x-b.x) + (a.y-b.y) * (a.y-b.y)).sqrt();
}
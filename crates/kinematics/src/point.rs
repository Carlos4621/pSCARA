#[derive(Debug, Clone, Copy)]
pub struct Point {
    pub x: f64,
    pub y: f64,
}

impl Point {
    pub fn distance_from_origin(&self) -> f64 {
        libm::hypot(self.x, self.y)
    }
}

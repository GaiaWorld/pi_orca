use std::ops::{Add, AddAssign, Div, DivAssign, Mul, MulAssign, Neg, Sub, SubAssign};
use wasm_bindgen::prelude::wasm_bindgen;

pub const RVO_EPSILON: f32 = 0.00001;

#[wasm_bindgen]
#[derive(Clone, Copy, Debug, Default)]
pub struct Vector2 {
    pub x: f32,
    pub y: f32,
}

#[wasm_bindgen]
impl Vector2 {
    pub fn default() -> Self {
        Self {
            x: Default::default(),
            y: Default::default(),
        }
    }

    pub fn new(x: f32, y: f32) -> Self {
        Self { x, y }
    }

    pub fn x(&self) -> f32 {
        self.x
    }

    pub fn y(&self) -> f32 {
        self.y
    }

    pub fn add(&self, other: &Vector2) -> Vector2 {
        Self {
            x: self.x + other.x,
            y: self.y + other.y,
        }
    }

    pub fn sub(&self, other: &Vector2) -> Vector2 {
        Self {
            x: self.x - other.x,
            y: self.y - other.y,
        }
    }

    pub fn mul(&self, other: &Vector2) -> f32 {
        self.x * other.x + self.y * other.y
    }

    pub fn mul_number(&self, other: f32) -> Vector2 {
        Self {
            x: self.x * other,
            y: self.y * other,
        }
    }

    pub fn div(&self, other: f32) -> Vector2 {
        Self {
            x: self.x / other,
            y: self.y / other,
        }
    }

    pub fn neg(&self) -> Vector2 {
        Self {
            x: -self.x,
            y: -self.y,
        }
    }

    pub fn abs(v: &Vector2) -> f32 {
        return ((*v) * (*v)).sqrt();
    }

    pub fn abs_sq(v: &Vector2) -> f32 {
        return (*v) * (*v);
    }

    pub fn det(v1: &Vector2, v2: &Vector2) -> f32 {
        return v1.x() * v2.y() - v1.y() * v2.x();
    }

    pub fn normalize(vector: &Vector2) -> Vector2 {
        return (*vector) / Self::abs(vector);
    }

    /**
     * \brief      Computes the squared distance from a line segment with the
     *             specified endpoints to a specified point.
     * \param      a               The first endpoint of the line segment.
     * \param      b               The second endpoint of the line segment.
     * \param      c               The point to which the squared distance is to
     *                             be calculated.
     * \return     The squared distance from the line segment to the point.
     */
    pub fn dist_sq_point_line_segment(a: &Vector2, b: &Vector2, c: &Vector2) -> f32 {
        let r = (((*c) - *(a)) * ((*b) - *(a))) / Self::abs_sq(&((*b) - *(a)));

        if r < 0.0 {
            return Self::abs_sq(&((*c) - *a));
        } else if r > 1.0 {
            return Self::abs_sq(&((*c) - *b));
        } else {
            return Self::abs_sq(&(*c - ((*b - *a) * r + a)));
        }
    }

    /**
     * \brief      Computes the signed distance from a line connecting the
     *             specified points to a specified point.
     * \param      a               The first point on the line.
     * \param      b               The second point on the line.
     * \param      c               The point to which the signed distance is to
     *                             be calculated.
     * \return     Positive when the point c lies to the left of the line ab.
     */
    pub fn left_of(a: &Vector2, b: &Vector2, c: &Vector2) -> f32 {
        return Self::det(&(*a - *c), &(*b - *a));
    }

    /**
     * \brief      Computes the square of a float.
     * \param      a               The float to be squared.
     * \return     The square of the float.
     */
    pub fn sqr(a: f32) -> f32 {
        return a * a;
    }
}

impl Neg for Vector2 {
    type Output = Self;

    fn neg(self) -> Self::Output {
        Self {
            x: -self.x,
            y: -self.y,
        }
    }
}

impl Mul for Vector2 {
    type Output = f32;

    fn mul(self, rhs: Self) -> f32 {
        return self.x * rhs.x + self.y * rhs.y;
    }
}

impl Mul<&Self> for Vector2 {
    type Output = f32;

    fn mul(self, rhs: &Self) -> f32 {
        
        return self.x * rhs.x + self.y * rhs.y;
    }
}

impl Mul<f32> for Vector2 {
    type Output = Self;

    fn mul(self, rhs: f32) -> Self {
        return Vector2::new(self.x * rhs, self.y * rhs);
    }
}

impl Div<f32> for Vector2 {
    type Output = Self;

    fn div(self, rhs: f32) -> Self::Output {
        let inv_s = 1.0 / rhs;
        return Vector2::new(self.x * inv_s, self.y * inv_s);
    }
}

impl Add<&Self> for Vector2 {
    type Output = Self;

    fn add(self, other: &Self) -> Self {
        Self {
            x: self.x + other.x,
            y: self.y + other.y,
        }
    }
}

impl Add for Vector2 {
    type Output = Self;

    fn add(self, other: Self) -> Self {
        Self {
            x: self.x + other.x,
            y: self.y + other.y,
        }
    }
}

impl Sub for Vector2 {
    type Output = Self;

    fn sub(self, rhs: Self) -> Self::Output {
        return Vector2::new(self.x - rhs.x, self.y - rhs.y);
    }
}

impl Sub<&Self> for Vector2 {
    type Output = Self;

    fn sub(self, rhs: &Self) -> Self::Output {
        return Vector2::new(self.x - rhs.x, self.y - rhs.y);
    }
}

impl PartialEq for Vector2 {
    fn eq(&self, other: &Self) -> bool {
        self.x == other.x && self.y == other.y
    }
}

impl MulAssign<f32> for Vector2 {
    fn mul_assign(&mut self, rhs: f32) {
        self.x *= rhs;
        self.y *= rhs;
    }
}

impl DivAssign<f32> for Vector2 {
    fn div_assign(&mut self, rhs: f32) {
        self.x /= rhs;
        self.y /= rhs;
    }
}

impl AddAssign<f32> for Vector2 {
    fn add_assign(&mut self, rhs: f32) {
        self.x += rhs;
        self.y += rhs;
    }
}

impl SubAssign<f32> for Vector2 {
    fn sub_assign(&mut self, rhs: f32) {
        self.x -= rhs;
        self.y -= rhs;
    }
}

impl Add<f32> for Vector2 {
    type Output = Self;

    fn add(self, other: f32) -> Self {
        Self {
            x: self.x + other,
            y: self.y + other,
        }
    }
}

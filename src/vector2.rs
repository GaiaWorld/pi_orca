use std::ops::{Add, AddAssign, Div, DivAssign, Mul, MulAssign, Neg, Sub, SubAssign};

pub const RVO_EPSILON: f32 = 0.00001;
use serde::{Deserialize, Serialize};

#[derive(Clone, Copy, Debug, Default, Serialize, Deserialize)]
pub struct Vector2 {
    pub x: f32,
    pub y: f32,
}

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

    /// 加上一个向量
    pub fn add(&self, other: &Vector2) -> Vector2 {
        Self {
            x: self.x + other.x,
            y: self.y + other.y,
        }
    }

    /// 减掉一个矢量
    pub fn sub(&self, other: &Vector2) -> Vector2 {
        Self {
            x: self.x - other.x,
            y: self.y - other.y,
        }
    }

    /// 乘以一个矢量
    pub fn mul(&self, other: &Vector2) -> f32 {
        self.x * other.x + self.y * other.y
    }

    /**
     * \relates    Vector2
     * \brief      计算指定二维向量与指定标量值的标量乘积。
     * \param      other           应计算标量乘法的标量值。    
     * \return     二维向量与标量值的标量乘法。
     */
    pub fn mul_number(&self, other: f32) -> Vector2 {
        Self {
            x: self.x * other,
            y: self.y * other,
        }
    }

    /// 除以一个标量
    pub fn div(&self, other: f32) -> Vector2 {
        Self {
            x: self.x / other,
            y: self.y / other,
        }
    }

    /// 反转一个矢量
    pub fn neg(&self) -> Vector2 {
        Self {
            x: -self.x,
            y: -self.y,
        }
    }

    /**
     * \relates    Vector2
     * \brief      计算指定二维向量的长度。
     * \param      vector 要计算其长度的二维向量。       
     * \return     二维向量的长度。
     */
    pub fn abs(v: &Vector2) -> f32 {
        return ((*v) * (*v)).sqrt();
    }

    /**
     * \relates    Vector2
     * \brief      计算指定二维向量的长度平方。
     * \param      要计算其长度平方的二维向量。          
     * \return     二维向量的平方长度。
     */
    pub fn abs_sq(v: &Vector2) -> f32 {
        return (*v) * (*v);
    }

    /**
     * \relates    Vector2
     * \brief      计算由指定二维向量组成的行列式二维方阵。
     * \param      vector1         二维方阵的顶行。
     * \param      vector2         二维方阵的底行。
     * \return     二维方阵的行列式。
     */
    pub fn det(v1: &Vector2, v2: &Vector2) -> f32 {
        return v1.x() * v2.y() - v1.y() * v2.x();
    }

    /**
     * \relates    Vector2
     * \brief      计算指定二维向量的归一化。
     * \param      vector  要计算归一化的二维向量。         
     * \return     二维向量的归一化。
     */
    pub fn normalize(vector: &Vector2) -> Vector2 {
        return (*vector) / Self::abs(vector);
    }

    /**
     * \brief      计算从具有指定端点的线段到指定点的平方距离。
     * \param      a               线段的第一个端点。
     * \param      b               线段的第二个端点。
     * \param      c               要计算平方距离的点。
     * \return     从线段到点的平方距离。
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
     * \brief      计算从连接指定点的线到指定点的有符号距离。
     * \param      a               线上的第一个点。
     * \param      b               线上的第二个点。
     * \param      c               要计算符号距离的点。
     * \return     当点 c 位于直线 ab 的左侧时为正。
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

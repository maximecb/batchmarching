use std::ops::{Add, Sub, Mul, Neg};
use std::f32::consts::PI;

// Degrees to radians
pub fn deg2rad(a: f32) -> f32
{
    (a * PI) / 180.0
}

// 3D vector
#[derive(Default, Copy, Clone, Debug, PartialEq)]
pub struct Vec3
{
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Vec3
{
    pub fn new(x: f32, y: f32, z: f32) -> Vec3
    {
        Self {
            x,
            y,
            z
        }
    }

    // Compute the length/norm
    pub fn norm(&self) -> f32
    {
        (self.x * self.x + self.y * self.y + self.z * self.z).sqrt()
    }

    // Compute a normalized vector
    pub fn normalized(&self) -> Vec3
    {
        let l = self.norm();

        if (l == 0.0)
        {
            return self.clone()
        }

        Vec3 {
            x: self.x / l,
            y: self.y / l,
            z: self.z / l,
        }
    }

    // Dot product
    pub fn dot(self, other: Vec3) -> f32
    {
        self.x * other.x + self.y * other.y + self.z * other.z
    }

    // Cross product
    pub fn cross(self, other: Vec3) -> Vec3
    {
        Vec3 {
            x: self.y * other.z - self.z * other.y,
            y: self.z * other.x - self.x * other.z,
            z: self.x * other.y - self.y * other.x,
        }
    }

    // Minimum
    pub fn min(self, other: Vec3) -> Vec3
    {
        Vec3 {
            x: if self.x < other.x { self.x } else { other.x },
            y: if self.y < other.y { self.y } else { other.y },
            z: if self.z < other.z { self.z } else { other.z },
        }
    }

    // Maximum
    pub fn max(self, other: Vec3) -> Vec3
    {
        Vec3 {
            x: if self.x > other.x { self.x } else { other.x },
            y: if self.y > other.y { self.y } else { other.y },
            z: if self.z > other.z { self.z } else { other.z },
        }
    }
}

impl Add for Vec3
{
    type Output = Vec3;

    fn add(self, other: Vec3) -> Vec3
    {
        Vec3 {
            x: self.x + other.x,
            y: self.y + other.y,
            z: self.z + other.z,
        }
    }
}

impl Sub for Vec3
{
    type Output = Vec3;

    fn sub(self, other: Vec3) -> Vec3
    {
        Vec3 {
            x: self.x - other.x,
            y: self.y - other.y,
            z: self.z - other.z,
        }
    }
}

impl Mul<f32> for Vec3
{
    type Output = Vec3;

    fn mul(self, scalar: f32) -> Vec3
    {
        Vec3 {
            x: self.x * scalar,
            y: self.y * scalar,
            z: self.z * scalar,
        }
    }
}

impl Mul<Vec3> for f32
{
    type Output = Vec3;

    fn mul(self, vec: Vec3) -> Vec3
    {
        Vec3 {
            x: self * vec.x,
            y: self * vec.y,
            z: self * vec.z,
        }
    }
}

impl Neg for Vec3
{
    type Output = Vec3;

    fn neg(self) -> Vec3
    {
        Vec3 {
            x: -self.x,
            y: -self.y,
            z: -self.z,
        }
    }
}

// 4x4 matrix
// Matrices are indexed as mat[row][col]
// This is a row-major ordering, such that
// each row is contiguous in memory
#[derive(Debug, Copy, Clone)]
pub struct Mat44 ([[f32; 4]; 4]);

// Identity matrix constant
const MAT44_IDENT: Mat44 = Mat44([
    [1.0, 0.0, 0.0, 0.0],
    [0.0, 1.0, 0.0, 0.0],
    [0.0, 0.0, 1.0, 0.0],
    [0.0, 0.0, 0.0, 1.0],
]);

impl Mat44
{
    // Generate a translation matrix from a 3D vector
    pub fn translate(v: Vec3) -> Mat44
    {
        Mat44([
            [1.0, 0.0, 0.0, v.x],
            [0.0, 1.0, 0.0, v.y],
            [0.0, 0.0, 1.0, v.z],
            [0.0, 0.0, 0.0, 1.0],
        ])
    }

    // Function to generate a rotation matrix around the x-axis
    pub fn rotate_x(theta: f32) -> Mat44
    {
        let sin_theta = theta.sin();
        let cos_theta = theta.cos();

        Mat44([
            [1.0, 0.0, 0.0, 0.0],
            [0.0, cos_theta, -sin_theta, 0.0],
            [0.0, sin_theta, cos_theta, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ])
    }

    // Rotation about the Y-axis
    pub fn rotate_y(theta: f32) -> Mat44
    {
        let cos_theta = theta.cos();
        let sin_theta = theta.sin();

        Mat44([
            [cos_theta, 0.0, -sin_theta, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [sin_theta, 0.0, cos_theta, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ])
    }

    // Generate a rotation matrix about the Z-axis
    pub fn rotate_z(theta: f32) -> Mat44
    {
        let cos_theta = theta.cos();
        let sin_theta = theta.sin();

        Mat44([
            [cos_theta, -sin_theta, 0.0, 0.0],
            [sin_theta, cos_theta, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ])
    }

    // Matrix multiplication
    pub fn mat_mul(self, other: Mat44) -> Mat44
    {
        let mut result = Mat44([[0.0; 4]; 4]);

        // For each row
        for i in 0..4
        {
            // For each column
            for j in 0..4
            {
                // Dot product
                for k in 0..4
                {
                    result.0[i][j] += self.0[i][k] * other.0[k][j];
                }
            }
        }

        result
    }

    // Transform a Vec3 using a 4x4 matrix
    pub fn transform(&self, v: Vec3) -> Vec3
    {
        let x = self.0[0][0] * v.x + self.0[0][1] * v.y + self.0[0][2] * v.z + self.0[0][3];
        let y = self.0[1][0] * v.x + self.0[1][1] * v.y + self.0[1][2] * v.z + self.0[1][3];
        let z = self.0[2][0] * v.x + self.0[2][1] * v.y + self.0[2][2] * v.z + self.0[2][3];

        Vec3 { x, y, z }
    }

    // TODO: this needs to be tested/validated
    //
    // Generate a perspective projection matrix
    // fov is the horizontal FOV angle
    pub fn perspective(fov: f32, aspect_ratio: f32, near: f32, far: f32) -> Mat44
    {
        let tan_half_fov = (fov / 2.0).tan();
        let cot_half_fov = 1.0 / tan_half_fov;

        let sx = 1.0 / (aspect_ratio * tan_half_fov);
        let sy = 1.0 / tan_half_fov;
        let sz = -(far + near) / (far - near);
        let pz = -(2.0 * far * near) / (far - near);

        Mat44([
            [sx, 0.0, 0.0, 0.0],
            [0.0, sy, 0.0, 0.0],
            [0.0, 0.0, sz, pz],
            [0.0, 0.0, -1.0, 0.0],
        ])
    }

    // TODO: this needs to be tested/validated
    //
    // Generate a lookat matrix
    // Projects from world space to view space
    pub fn look_at(eye: Vec3, target: Vec3, up: Vec3) -> Mat44
    {
        // Compute forward, right, up vectors
        let f = (target - eye).normalized();
        let r = up.cross(f).normalized();
        let u = f.cross(r);

        let m = Mat44([
            [r.x, r.y, r.z, 0.0],
            [u.x, u.y, u.z, 0.0],
            [-f.x, -f.y, -f.z, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ]);

        let translation = Mat44::translate(-eye);

        // Combine the rotation and translation
        m * translation
    }
}

// Operator overload for matrix multiplication
impl Mul for Mat44
{
    type Output = Mat44;

    fn mul(self, other: Mat44) -> Mat44
    {
        self.mat_mul(other)
    }
}

use std::time::SystemTime;
use std::f32::INFINITY;
use crate::math::*;

pub struct Image
{
    width: usize,
    height: usize,

    // RGBA32 pixel data
    data: Vec<u32>,
}

pub fn rgb32(r: f32, g: f32, b: f32) -> u32
{
    let r = ((r.min(1.0) * 255.0) as u32) & 255;
    let g = ((g.min(1.0) * 255.0) as u32) & 255;
    let b = ((b.min(1.0) * 255.0) as u32) & 255;
    return (0xFF_00_00_00) | (r << 16) | (g << 8) | (b << 0);
}

impl Image
{
    pub fn new(width: usize, height: usize) -> Image
    {
        let mut data = Vec::new();
        data.resize(width * height, 0);

        Image {
            width,
            height,
            data,
        }
    }

    pub fn get_width(&self) -> usize
    {
        self.width
    }

    pub fn clear(&mut self, r: f32, g: f32, b: f32)
    {
        let color = rgb32(r, g, b);
        self.data.fill(color);
    }

    pub fn set_pixel(&mut self, x: usize, y: usize, r: f32, g: f32, b: f32)
    {
        assert!(x < self.width);
        assert!(y < self.height);
        self.data[y * self.width + x] = rgb32(r, g, b);
    }

    pub unsafe fn raw_data(&self) -> &[u8]
    {
        let ptr = self.data.as_ptr();
        let ptr = ptr as *const u8;
        let num_bytes = self.data.len() * 4;
        std::slice::from_raw_parts(ptr, num_bytes)
    }
}

static mut EVAL_COUNT: u64 = 0;
static mut TIME: f32 = 0.0;

fn sd_torus(p: Vec3, tx: f32, ty: f32) -> f32
{
    let qx: f32 = (p.x * p.x + p.z * p.z).sqrt() - tx;
    return (qx*qx + p.y * p.y).sqrt() - ty;
}

fn sdf(p: Vec3) -> f32
{
    unsafe { EVAL_COUNT += 1 };

    // Generating rotation matrices here is inefficient
    // since it only really needs to happen once per frame,
    // but I wanted to keep the code simple
    let t = unsafe { TIME };
    let rot_x = Mat44::rotate_x(0.50 * t);
    let rot_z = Mat44::rotate_z(0.35 * t);
    let rot_p = (rot_x * rot_z).transform(p);

    sd_torus(rot_p, 40.0, 20.0)
}

// Estimate the surface normal with 4 SDF evaluations. Based
// on an article by the legendary Inigo Quilez:
// https://iquilezles.org/articles/normalsSDF/
fn calc_normal(p: Vec3) -> Vec3
{
    let h = 0.001;
    let xyy = Vec3::new( 1.0, -1.0, -1.0);
    let yyx = Vec3::new(-1.0, -1.0,  1.0);
    let yxy = Vec3::new(-1.0,  1.0, -1.0);
    let xxx = Vec3::new( 1.0,  1.0,  1.0);

    return (
      xyy * sdf(p + xyy * h) +
      yyx * sdf(p + yyx * h) +
      yxy * sdf(p + yxy * h) +
      xxx * sdf(p + xxx * h)
    ).normalized();
}

// Returns a distance value
// Infinity if no intersection
fn march_ray(cam_pos: Vec3, ray_dir: Vec3, pixel_size_ratio: f32) -> f32
{
    const MAX_STEPS: usize = 100;
    const MAX_DIST: f32 = 400.0;

    let mut t = 0.0;
    let mut num_steps = 0;

    while num_steps < MAX_STEPS && t < MAX_DIST {
        let cur_pos = cam_pos + ray_dir * t;

        // Query the distance function
        let dist = sdf(cur_pos);

        let epsilon = (t * pixel_size_ratio).max(0.001);

        if dist < epsilon {
            break;
        }

        t += dist;
        num_steps += 1;
    }

    if num_steps == MAX_STEPS || t > MAX_DIST {
        return INFINITY;
    }

    return t;
}

// Returns a distance value
// Infinity if no intersection
// Accelerated sphere tracing algorithm (Balint & Valasek 2018)
fn march_ray_accel(cam_pos: Vec3, ray_dir: Vec3, pixel_size_ratio: f32) -> f32
{
    const MAX_STEPS: usize = 100;
    const MAX_DIST: f32 = 400.0;

    // Relaxation parameter (over-relaxation factor = 1 + w)
    let w = 0.9;

    let mut r_m1 = 0.0;
    let mut r_i = sdf(cam_pos);
    let mut d_i = r_i;
    let mut t = 0.0;
    let mut num_steps = 0;

    while num_steps < MAX_STEPS && t < MAX_DIST {
        let epsilon = (t * pixel_size_ratio).max(0.001);
        if r_i < epsilon {
            return t;
        }

        // Compute next step distance using over-relaxation
        if num_steps > 0 {
            let denom = d_i + r_m1 - r_i;
            if denom.abs() > 1e-6 {
                d_i = r_i * (1.0 + w * (d_i - r_m1 + r_i) / denom);
            } else {
                d_i = r_i;
            }
        } else {
            d_i = r_i;
        }

        // Safety check to ensure we don't go backwards or get stuck
        if d_i < 0.001 { d_i = r_i; }

        let mut r_p1 = sdf(cam_pos + (t + d_i) * ray_dir);

        // If the unbounding spheres are disjoint, backtrack to standard sphere tracing
        if d_i > r_i + r_p1
        {
            d_i = r_i;
            r_p1 = sdf(cam_pos + (t + d_i) * ray_dir);
        }

        let next_epsilon = ((t + d_i) * pixel_size_ratio).max(0.001);
        if r_p1 < next_epsilon {
            return t + d_i;
        }

        t += d_i;
        num_steps += 1;

        r_m1 = r_i;
        r_i = r_p1;
    }

    if t > MAX_DIST {
        return INFINITY;
    }

    return if r_i < (t * pixel_size_ratio).max(0.001) { t } else { INFINITY };
}

#[derive(Debug, Copy, Clone, PartialEq)]
pub enum RenderMethod
{
    Standard,
    Accelerated,
    Batch,
}

fn shade_pixel(
    frame: &mut Image,
    x: usize,
    y: usize,
    cam_pos: Vec3,
    ray_dir: Vec3,
    t: f32,
) {
    if t == INFINITY {
        return;
    }
    let p = cam_pos + ray_dir * t;
    let light_dir: Vec3 = Vec3::new(1.0, 1.3, -1.0).normalized();
    let normal = calc_normal(p);
    let dot = normal.dot(light_dir);
    let cos_theta = if dot < 0.0 { -dot } else { 0.0 };
    let brightness = (0.20 + cos_theta).min(1.0);
    frame.set_pixel(x, y, brightness, 0.0, 0.0);
}

fn render_rect(
    frame: &mut Image,
    cam_pos: Vec3,
    top_left: Vec3,
    right: Vec3,
    up: Vec3,
    pixel_size_ratio: f32,
    half_w: f32,
    half_h: f32,
    x: usize,
    y: usize,
    w: usize,
    h: usize,
    mut t: f32,
) {
    const MAX_DIST: f32 = 400.0;
    const MAX_STEPS: usize = 100;
    const EPSILON_BASE: f32 = 0.001;

    if t > MAX_DIST {
        return;
    }

    // Center of this rect in pixel coordinates
    let cx = x as f32 + w as f32 / 2.0;
    let cy = y as f32 + h as f32 / 2.0;

    // Ray direction for the center
    let x_ratio = cx / (frame.width as f32);
    let y_ratio = cy / (frame.height as f32);
    let pix_pos = top_left + (2.0 * half_w) * x_ratio * right + (2.0 * half_h) * y_ratio * -up;
    let ray_dir = (pix_pos - cam_pos).normalized();

    // Spread: max distance from center ray to any point in the quad at distance 1.0
    let dx = w as f32 / 2.0;
    let dy = h as f32 / 2.0;
    let dr = (dx*dx + dy*dy).sqrt();
    let spread = dr * pixel_size_ratio;

    if w == 1 && h == 1 {
        // Single pixel - march until hit or max dist
        let mut num_steps = 0;
        let epsilon_ratio = 0.5 * 1.414 * pixel_size_ratio;

        while num_steps < MAX_STEPS && t < MAX_DIST {
            let p = cam_pos + ray_dir * t;
            let d = sdf(p);

            let epsilon = (t * epsilon_ratio).max(EPSILON_BASE);

            if d < epsilon {
                shade_pixel(frame, x, y, cam_pos, ray_dir, t);
                return;
            }

            t += d;
            num_steps += 1;
        }
        return;
    }

    let mut num_steps = 0;
    while num_steps < MAX_STEPS && t < MAX_DIST {
        let p = cam_pos + ray_dir * t;
        let d = sdf(p);

        let radius = t * spread;
        let epsilon = (t * pixel_size_ratio).max(EPSILON_BASE);

        if d > radius + epsilon {
            // Safe to advance the whole quad
            let dt = (d - radius) / (1.0 + spread);
            t += dt.max(epsilon);
            num_steps += 1;
        } else {
            // Too close, must recurse
            let w1 = w / 2;
            let w2 = w - w1;
            let h1 = h / 2;
            let h2 = h - h1;

            if w1 > 0 && h1 > 0 { render_rect(frame, cam_pos, top_left, right, up, pixel_size_ratio, half_w, half_h, x, y, w1, h1, t); }
            if w2 > 0 && h1 > 0 { render_rect(frame, cam_pos, top_left, right, up, pixel_size_ratio, half_w, half_h, x + w1, y, w2, h1, t); }
            if w1 > 0 && h2 > 0 { render_rect(frame, cam_pos, top_left, right, up, pixel_size_ratio, half_w, half_h, x, y + h1, w1, h2, t); }
            if w2 > 0 && h2 > 0 { render_rect(frame, cam_pos, top_left, right, up, pixel_size_ratio, half_w, half_h, x + w1, y + h1, w2, h2, t); }
            return;
        }
    }
}

pub fn render_scene(
    frame: &mut Image,
    cam_dist: f32,
    rot_z: f32,
    rot_x: f32,
    fov_x: f32,
    dt: f32,
    method: RenderMethod,
)
{
    let start_time = SystemTime::now();

    frame.clear(0.1, 0.1, 0.1);

    // Camera rotation matrix
    let rotation = (
        Mat44::rotate_z(deg2rad(-rot_z)) *
        Mat44::rotate_x(deg2rad(rot_x))
    );

    // Rotate the camera around the object
    let cam_pos = rotation.transform(Vec3::new(0.0, -cam_dist, 0.0));

    // Compute camera vectors
    // Camera looks into +Y (into the screen)
    // +X goes to the right
    // +Z points up
    let forward = rotation.transform(Vec3::new(0.0, 1.0, 0.0));
    let right = rotation.transform(Vec3::new(1.0, 0.0, 0.0));
    let up = rotation.transform(Vec3::new(0.0, 0.0, 1.0));

    // Half of the image frame width
    // With the image plane being one unit away
    let half_w = deg2rad(fov_x / 2.0).tan();
    let half_h = half_w * (frame.height as f32) / (frame.width as f32);

    let pixel_size_ratio = (2.0 * half_w) / (frame.width as f32);

    // Compute the position of the top-left corner of the image plane
    let top_left = (cam_pos + forward) - (half_w * right) + (half_h * up);

    unsafe { EVAL_COUNT = 0 };
    unsafe { TIME += dt };

    match method {
        RenderMethod::Batch => {
            render_rect(
                frame,
                cam_pos,
                top_left,
                right,
                up,
                pixel_size_ratio,
                half_w,
                half_h,
                0, 0,
                frame.width, frame.height,
                0.0
            );
        }
        RenderMethod::Standard | RenderMethod::Accelerated => {
            for y in 0..frame.height {
                for x in 0..frame.width {
                    let x_ratio = (x as f32 + 0.5) / (frame.width as f32);
                    let y_ratio = (y as f32 + 0.5) / (frame.height as f32);
                    let pix_pos = top_left + (2.0 * half_w) * x_ratio * right + (2.0 * half_h) * y_ratio * -up;
                    let ray_dir = (pix_pos - cam_pos).normalized();

                    let t = match method {
                        RenderMethod::Standard => march_ray(cam_pos, ray_dir, pixel_size_ratio),
                        RenderMethod::Accelerated => march_ray_accel(cam_pos, ray_dir, pixel_size_ratio),
                        _ => unreachable!(),
                    };

                    shade_pixel(frame, x, y, cam_pos, ray_dir, t);
                }
            }
        }
    }

    let end_time = SystemTime::now();
    let dt = end_time.duration_since(start_time).unwrap().as_millis();
    let fps = 1.0 / ((dt as f32) / 1000.0);
    println!("Render time: {} ms, FPS: {:.1} ({:?} ray marching)", dt, fps, method);

    let evals_per_pix = unsafe { EVAL_COUNT as f32 } / (frame.height * frame.width) as f32;
    println!("SDF evals/pixel : {:.2}", evals_per_pix);
}


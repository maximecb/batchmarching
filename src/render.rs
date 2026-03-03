use std::time::SystemTime;
use std::f64::INFINITY;
use crate::math::*;

pub struct Image
{
    width: usize,
    height: usize,

    // RGBA32 pixel data
    data: Vec<u32>,
}

pub fn rgb32(r: f64, g: f64, b: f64) -> u32
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

    pub fn clear(&mut self, r: f64, g: f64, b: f64)
    {
        let color = rgb32(r, g, b);
        self.data.fill(color);
    }

    pub fn set_pixel(&mut self, x: usize, y: usize, r: f64, g: f64, b: f64)
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

fn sd_torus(p: Vec3, tx: f64, ty: f64) -> f64
{
    let qx: f64 = (p.x * p.x + p.z * p.z).sqrt() - tx;
    return (qx*qx + p.y * p.y).sqrt() - ty;
}

fn sdf(p: Vec3) -> f64
{
    unsafe { EVAL_COUNT += 1 };


    sd_torus(p, 40.0, 20.0)



}

// Estimate the surface normal with 4 SDF evaluations. Based
// on an article by the legendary Inigo Quilez:
// https://iquilezles.org/articles/normalsSDF/
fn calc_normal(p: Vec3) -> Vec3
{
    let h = 0.000001;
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

/*
// Estimate the extents of the object represented by an SDF
// NOTE: if we just want a bounding sphere, then we should
// sample random points close to the sphere edge
// Take a random vector and normalize it.
pub fn estimate_bounds(vm: &mut VM, sdf_fn: Value) -> (Vec3, Vec3)
{
    use rand::{thread_rng};
    use rand_distr::{Normal, Distribution};

    let mut min = Vec3::default();
    let mut max = Vec3::default();

    let mut num_samples: usize = 0;

    while (num_samples < 50_000)
    {
        let norm = (max - min).norm();
        let gaussian = Normal::new(0.0, norm / 2.0 + 0.01).unwrap();

        let mut pos = Vec3::new(
            gaussian.sample(&mut rand::thread_rng()),
            gaussian.sample(&mut rand::thread_rng()),
            gaussian.sample(&mut rand::thread_rng()),
        );

        let d = vm.call_sdf(sdf_fn, pos);

        if d < 0.0
        {
            let new_min = min.min(pos);
            let new_max = max.max(pos);

            if new_min != min || new_max != max {
                min = new_min;
                max = new_max;

                println!("itr #{}", num_samples);
                println!("min {:?}", min);
                println!("max {:?}", max);
            }
        }

        num_samples += 1;
    }

    return (min, max);
}
*/

// Returns a distance value
// Infinity if no intersection
fn march_ray(cam_pos: Vec3, ray_dir: Vec3, pixel_size_ratio: f64) -> f64
{
    const MAX_STEPS: usize = 100;

    // 2,500 mm (2.5m)
    // TODO: adjust dynamically based on object size
    const MAX_DIST: f64 = 2500.0;

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
fn march_ray_accel(cam_pos: Vec3, ray_dir: Vec3, pixel_size_ratio: f64) -> f64
{
    const MAX_STEPS: usize = 100;

    // 2,500 mm (2.5m)
    // TODO: adjust dynamically based on object size
    const MAX_DIST: f64 = 2500.0;

    // Relaxation parameter
    let w = 0.9;

    let mut r_m1 = 0.0;
    let mut r_i = 0.0;
    let mut d_i = 0.0;
    let mut t = 0.0;
    let mut num_steps = 0;

    while num_steps < MAX_STEPS && t < MAX_DIST {
        d_i = r_i + w * r_i * (d_i - r_m1 + r_i) / (d_i + r_m1 - r_i);

        let mut r_p1 = sdf(cam_pos + (t + d_i) * ray_dir);

        // If the unbounding spheres are disjoint, backtrack
        if d_i > r_i + r_p1
        {
            d_i = r_i;
            r_p1 = sdf(cam_pos + (t + d_i) * ray_dir);
        }

        let epsilon = ((t + d_i) * pixel_size_ratio).max(0.001);

        if r_p1 < epsilon {
            break;
        }

        t += d_i;
        num_steps += 1;

        r_m1 = r_i;
        r_i = r_p1;
    }

    if num_steps == MAX_STEPS || t > MAX_DIST {
        return INFINITY;
    }

    return t;
}

fn render_rect(
    frame: &mut Image,
    cam_pos: Vec3,
    top_left: Vec3,
    right: Vec3,
    up: Vec3,
    pixel_size_ratio: f64,
    half_w: f64,
    half_h: f64,
    x: usize,
    y: usize,
    w: usize,
    h: usize,
    mut t: f64,
) {
    const MAX_DIST: f64 = 2500.0;
    const MAX_STEPS: usize = 150;
    const EPSILON_BASE: f64 = 0.001;

    if t > MAX_DIST {
        return;
    }

    // Center of this rect in pixel coordinates
    let cx = x as f64 + w as f64 / 2.0;
    let cy = y as f64 + h as f64 / 2.0;

    // Ray direction for the center
    let x_ratio = cx / (frame.width as f64);
    let y_ratio = cy / (frame.height as f64);
    let pix_pos = top_left + (2.0 * half_w) * x_ratio * right + (2.0 * half_h) * y_ratio * -up;
    let ray_dir = (pix_pos - cam_pos).normalized();

    // Spread: max distance from center ray to any point in the quad at distance 1.0
    let dx = w as f64 / 2.0;
    let dy = h as f64 / 2.0;
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
                // Hit!
                let light_dir: Vec3 = Vec3::new(1.0, 1.3, -1.0).normalized();
                let normal = calc_normal(p);
                let dot = normal.dot(light_dir);
                let cos_theta = if dot < 0.0 { -dot } else { 0.0 };
                let brightness = 0.35 + cos_theta;
                frame.set_pixel(x, y, brightness, 0.0, 0.0);
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
    cam_dist: f64,
    rot_z: f64,
    rot_x: f64,
    fov_x: f64,
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
    let half_h = half_w * (frame.height as f64) / (frame.width as f64);

    let pixel_size_ratio = (2.0 * half_w) / (frame.width as f64);

    // Compute the position of the top-left corner of the image plane
    let top_left = (cam_pos + forward) - (half_w * right) + (half_h * up);

    unsafe { EVAL_COUNT = 0 };

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

    let end_time = SystemTime::now();
    let dt = end_time.duration_since(start_time).unwrap().as_millis();
    println!("render time: {} ms", dt);

    let evals_per_pix = unsafe { EVAL_COUNT as f64 } / (frame.height * frame.width) as f64;
    println!("evals/pixel : {:.2}", evals_per_pix);
}

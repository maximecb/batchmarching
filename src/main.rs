#![allow(dead_code)]
#![allow(unused_variables)]
#![allow(unused_parens)]
#![allow(unused_mut)]

extern crate sdl2;
mod math;
mod render;
use sdl2::pixels::Color;
use sdl2::event::Event;
use sdl2::keyboard::Keycode;
use sdl2::render::Texture;
use sdl2::render::TextureAccess;
use sdl2::pixels::PixelFormatEnum;
use sdl2::video::Window;
use sdl2::render::Canvas;
use sdl2::sys::SDL_WindowFlags;
use crate::render::*;

// Show one frame of image data in a window
pub fn show_frame(
    canvas: &mut Canvas<Window>,
    texture: &mut Texture,
    image: &Image
)
{
    let window = canvas.window_mut();

    // If no frame has been drawn yet
    if (window.window_flags() & SDL_WindowFlags::SDL_WINDOW_HIDDEN as u32) != 0 {
        // We show and raise the window at the moment the first frame is drawn
        // This avoids showing a blank window too early
        window.show();
        window.raise();
    }

    // Update the texture
    unsafe {
        let pitch = 4 * image.get_width();
        let image_bytes = image.raw_data();
        texture.update(None, image_bytes, pitch).unwrap();
    }

    // Copy the texture into the canvas
    canvas.copy(
        texture,
        None,
        None
    ).unwrap();

    // Update the screen with any rendering performed since the previous call
    canvas.present();
}

fn main()
{
    let window_width: u32 = 800;
    let window_height: u32 = 600;

    let sdl = sdl2::init().unwrap();
    let sdl_video = sdl.video().unwrap();

    let mut window = sdl_video.window("Adaptive Ray Batch Marching", window_width, window_height)
        .hidden()
        .position_centered()
        .build()
        .unwrap();

    let mut canvas = window.into_canvas().build().unwrap();

    canvas.set_draw_color(Color::RGB(0, 0, 0));
    canvas.clear();
    canvas.present();

    let mut texture_creator = canvas.texture_creator();

    // Creat the texture to render into
    // Pixels use the BGRA byte order (0xAA_RR_GG_BB on a little-endian machine)
    let mut texture = texture_creator.create_texture(
        PixelFormatEnum::BGRA32,
        TextureAccess::Streaming,
        window_width,
        window_height
    ).unwrap();

    let mut fb = Image::new(
        window_width as usize,
        window_height as usize,
    );

    let mut event_pump = sdl.event_pump().unwrap();

    let mut rot_z = 0.0;
    let mut rot_x = 0.0;
    let mut render_method = RenderMethod::Standard;

    let mut last_time = std::time::Instant::now();
    let mut stopped = false;

    loop
    {
        let current_time = std::time::Instant::now();
        let dt = if stopped {
            0.0
        } else {
            current_time.duration_since(last_time).as_secs_f32()
        };
        last_time = current_time;

        // See: https://docs.rs/sdl2/0.30.0/sdl2/event/enum.Event.htmls
        match event_pump.poll_event() {
            Some(Event::Quit { .. }) => {
                break;
            }

            Some(Event::KeyDown { window_id, keycode: Some(keycode), .. }) => {
                match keycode {
                    Keycode::Escape => {
                        break
                    }

                    Keycode::Num1 => {
                        render_method = RenderMethod::Standard;
                    }

                    Keycode::Num2 => {
                        render_method = RenderMethod::Accelerated;
                    }

                    Keycode::Num3 => {
                        render_method = RenderMethod::Batch;
                    }

                    Keycode::Num4 => {
                        render_method = RenderMethod::Approx;
                    }

                    Keycode::Space => {
                        stopped = !stopped;
                    }

                    _ => {}
                }
            }

            _ => {}
        }

        render_scene(
            &mut fb,
            200.0,
            rot_z,
            rot_x,
            60.0,
            dt,
            render_method,
        );

        show_frame(
            &mut canvas,
            &mut texture,
            &fb,
        );
    }
}

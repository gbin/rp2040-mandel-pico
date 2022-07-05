pub(crate) mod tools;

use crate::mandel::tools::Rectangle;
use fixed::traits::Fixed;
use fixed::types::{I16F16, I16F48, I2F14, I32F32, I4F12, I4F28, I9F7};

pub const MAX_MANDEL_ITERATION: i32 = 100;

pub const SCREEN_HEIGHT: u16 = 135;
pub const SCREEN_WIDTH: u16 = 240;
pub const GRAPHIC_BUFFER_SIZE: usize = (SCREEN_HEIGHT * SCREEN_WIDTH) as usize;

pub fn mandel(x: f64, y: f64) -> i32 {
    let mut u = 0.0;
    let mut v = 0.0;
    let mut u2 = 0.0;
    let mut v2 = 0.0;
    let mut k = 0;
    while k < MAX_MANDEL_ITERATION && (u2 + v2 < 4.0) {
        v = 2.0 * u * v + y;
        u = u2 - v2 + x;
        u2 = u * u;
        v2 = v * v;
        k = k + 1;
    }
    k
}

pub fn mandel_fp<T: fixed::traits::Fixed>(x: T, y: T) -> i32 {
    let mut u = T::ZERO;
    let mut v = T::ZERO;
    let mut u2 = T::ZERO;
    let mut v2 = T::ZERO;
    let mut iteration = 0;
    while iteration < MAX_MANDEL_ITERATION && (u2 + v2) < (T::from_num(4.0)) {
        v = T::from_num(2.0) * u * v + y;
        u = u2 - v2 + x;
        u2 = u * u;
        v2 = v * v;
        iteration = iteration + 1;
    }
    iteration
}

//
// Mandel is in the square:
// x between (-2.00, 0.47)
// y between (-1.12, 1.12)
/// Draw Mandel on an offscreen buffer.
/// The pixel target is only a filter to render a subpart of it (it doesn't after the ratio that is assuming a full screen rendering anyway)
/// mandel_region: the region to draw in mandel space (for the whole screen)
/// screen_region: the target part to draw
pub fn draw_on_buffer_inner<P: Fixed, M: Fixed>(
    mandel_region: Rectangle<P>,
    screen_region: Rectangle<u16>,
    buffer: &mut [u16; GRAPHIC_BUFFER_SIZE],
) {
    let m = mandel_region;
    let s = screen_region;
    let mut color: u16;
    for py in s.by..s.ey {
        let x: P = ((m.ex - m.bx) / P::from_num(SCREEN_WIDTH)) * P::from_num(py) + m.bx;
        for px in s.bx..s.ex {
            let y: P = ((m.ey - m.by) / P::from_num(SCREEN_HEIGHT)) * P::from_num(px) + m.by;
            let iteration = mandel_fp(M::from_fixed(x), M::from_fixed(y));
            if iteration == MAX_MANDEL_ITERATION {
                color = 0xffff;
            } else {
                color = (iteration % 0xffff) as u16;
            }
            buffer[(py * SCREEN_HEIGHT + px) as usize] = color;
        }
    }
}

pub fn draw_on_buffer_dispatch(
    mandel_region: Rectangle<f32>,
    screen_region: Rectangle<u16>,
    buffer: &mut [u16; GRAPHIC_BUFFER_SIZE],
) {
    if mandel_region.width() > 1.0 {
        draw_on_buffer_inner::<I16F16, I9F7>(
            Rectangle::<I16F16>::from(mandel_region),
            screen_region,
            buffer,
        );
        return;
    }
    if mandel_region.width() > 0.2 {
        draw_on_buffer_inner::<I16F16, I4F12>(
            Rectangle::<I16F16>::from(mandel_region),
            screen_region,
            buffer,
        );
        return;
    }

    draw_on_buffer_inner::<I16F16, I4F28>(
        Rectangle::<I16F16>::from(mandel_region),
        screen_region,
        buffer, );
}

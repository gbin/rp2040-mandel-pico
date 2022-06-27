use crate::FP;
use fixed::types::I16F16;

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
/// bx = top left x of mandel to draw
/// by = top left y of mandel to draw
/// ex = bottom right x of mandel to draw
/// ey = bottom right y of mandel to draw
/// pbx = target pixel top left coord x
/// pby = target pixel top right coord y
/// pex = target pixel bottom left coord x
/// pey = target pixel bottom right coord y
pub fn draw_on_buffer(
    bx: I16F16,
    by: I16F16,
    ex: I16F16,
    ey: I16F16,
    pbx: u16,
    pby: u16,
    pex: u16,
    pey: u16,
    buffer: &mut [u16; GRAPHIC_BUFFER_SIZE],
) {
    let mut color: u16;
    for py in pby..pey {
        let x: I16F16 = ((ex - bx) / I16F16::from_num(SCREEN_WIDTH)) * I16F16::from_num(py) + bx;
        for px in pbx..pex {
            let y: I16F16 =
                ((ey - by) / I16F16::from_num(SCREEN_HEIGHT)) * I16F16::from_num(px) + by;
            let iteration = mandel_fp(I16F16::to_num::<FP>(x), I16F16::to_num::<FP>(y));
            if iteration == MAX_MANDEL_ITERATION {
                color = 0xffff;
            } else {
                color = (iteration % 0xffff) as u16;
            }
            buffer[(py * SCREEN_HEIGHT + px) as usize] = color;
        }
    }
}

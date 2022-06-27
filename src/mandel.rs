use fixed::types::I16F16;
use crate::FP;

pub const MAX_MANDEL_ITERATION:i32 = 100;

pub const SCREEN_HEIGHT:u16 = 135;
pub const SCREEN_WIDTH:u16 = 240;
pub const GRAPHIC_BUFFER_SIZE:usize = (SCREEN_HEIGHT * SCREEN_WIDTH) as usize;

pub fn mandel(x:f64, y:f64) -> i32 {
    let mut u = 0.0;
    let mut v= 0.0;
    let mut u2 = 0.0;
    let mut v2 = 0.0;
    let mut k=0;
    while k< MAX_MANDEL_ITERATION && (u2+v2<4.0){
        v = 2.0 * u * v + y;
        u = u2 - v2 + x;
        u2 = u * u;
        v2 = v * v;
        k = k + 1;
    }
    k
}

pub fn mandel_fp<T:fixed::traits::Fixed>(x:T, y:T) -> i32 {
    let mut u = T::ZERO;
    let mut v= T::ZERO;
    let mut u2 = T::ZERO;
    let mut v2 = T::ZERO;
    let mut iteration =0;
    while iteration < MAX_MANDEL_ITERATION && (u2+v2)<(T::from_num(4.0)){
        v = T::from_num(2.0 ) * u * v + y;
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
pub fn draw_on_buffer(bx:I16F16, by:I16F16, ex:I16F16, ey:I16F16, buffer: &mut [u16; GRAPHIC_BUFFER_SIZE]) {
    let mut color: u16;
    for py in 0..SCREEN_WIDTH {
        let x: I16F16 = ((ex -bx) / I16F16::from_num(SCREEN_WIDTH)) * I16F16::from_num(py) + bx;
        for px in 0..SCREEN_HEIGHT {
            let y: I16F16 = ((ey - by) / I16F16::from_num(SCREEN_HEIGHT )) * I16F16::from_num(px) + by;
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

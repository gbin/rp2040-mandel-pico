pub const MAX_MANDEL_ITERATION:i32 = 100;

// Mandel is in the square:
// x between (-2.00, 0.47)
// y between (-1.12, 1.12)
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
    let mut k=0;
    while k< MAX_MANDEL_ITERATION && (u2+v2)<(T::from_num(4.0)){
        v = T::from_num(2.0 ) * u * v + y;
        u = u2 - v2 + x;
        u2 = u * u;
        v2 = v * v;
        k = k + 1;
    }
    k
}

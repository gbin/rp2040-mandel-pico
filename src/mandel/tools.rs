use crate::mandel::tools;
use fixed::traits::Fixed;

/// represents a 2-corner coordinates
#[derive(Clone, Copy)]
pub struct Rectangle<T> {
    pub(crate) bx: T,
    pub(crate) by: T,
    pub(crate) ex: T,
    pub(crate) ey: T,
}

impl<T> Rectangle<T> {
    pub const fn new(bx: T, by: T, ex: T, ey: T) -> Rectangle<T> {
        Rectangle::<T> { bx, by, ex, ey }
    }
}

impl Rectangle<f32> {
    pub const fn zero_f32() -> Rectangle<f32> {
        Rectangle::<f32> {
            bx: 0.0,
            by: 0.0,
            ex: 0.0,
            ey: 0.0,
        }
    }
}

impl<T: Fixed> Rectangle<T> {
    pub fn from(src: Rectangle<f32>) -> Rectangle<T> {
        Rectangle::<T> {
            bx: T::from_num(src.bx),
            by: T::from_num(src.by),
            ex: T::from_num(src.ex),
            ey: T::from_num(src.ey),
        }
    }
    pub const fn zero() -> Rectangle<T> {
        Rectangle::<T> {
            bx: T::ZERO,
            by: T::ZERO,
            ex: T::ZERO,
            ey: T::ZERO,
        }
    }
}

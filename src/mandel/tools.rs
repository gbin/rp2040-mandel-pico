use crate::mandel::tools;
use fixed::traits::Fixed;

/// represents a 2-corner coordinates
#[derive(Clone, Copy, Default)]
pub struct Rectangle<T> {
    pub bx: T,
    pub by: T,
    pub ex: T,
    pub ey: T,
}

impl<T> Rectangle<T> {
    pub const fn new(bx: T, by: T, ex: T, ey: T) -> Rectangle<T> {
        Rectangle::<T> { bx, by, ex, ey }
    }
}

// We don't have the std so we cannot just add the substract Trait from std::
impl Rectangle<f32> {
    pub fn width(&self) -> f32 {
        self.ey - self.by
    }
}

impl Rectangle<u16> {
    pub fn width(&self) -> u16 {
        self.ey - self.by
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
}

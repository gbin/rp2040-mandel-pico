use crate::mandel::tools;
use fixed::traits::Fixed;

/// represents a 2-corner coordinates
#[derive(Clone, Copy, Default)]
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
    fn width(&mut self) -> f32 {
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

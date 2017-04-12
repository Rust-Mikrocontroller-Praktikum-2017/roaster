use core::ops::Sub;
use model::Range;

pub fn delta<T: Ord + Sub>(a: T, b: T) -> T::Output {
    if a < b {
        b - a
    } else {
        a - b
    }
}

pub fn clamp<T: PartialOrd>(value: T, min: T, max: T) -> T {
    if value > max { max }
    else if value < min { min }
    else { value }
}

pub fn clamp_range<T: PartialOrd>(value: T, range: Range<T>) -> T {
    clamp(value, range.from, range.to)
}
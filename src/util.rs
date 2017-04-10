use core::ops::Sub;

pub fn delta<T: Ord + Sub>(a: T, b: T) -> T::Output {
    if a < b {
        b - a
    } else {
        a - b
    }
}
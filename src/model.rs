use core::ops::Sub;
use lcd;
use time;

/// Temperature in degrees celsius
pub type Temperature = f32;
/// Time in seconds
pub type Time = f32;

#[derive(Copy, Clone)]
pub struct TimeTemp {
    pub time: Time,
    pub temp: Temperature,
}

#[derive(Clone, Copy)]
pub struct Range<T> {
    pub from: T,
    pub to: T
}

impl<T: PartialOrd + Sub + Copy + Into<f32>> Range<T> {
    pub fn new(from: T, to: T) -> Range<T> {
        Range {
            from: from,
            to: to,
        }
    }

    #[inline]
    pub fn size(&self) -> <T as Sub>::Output {
        if self.to < self.from {
            self.from - self.to
        } else {
            self.to - self.from
        }
    }

    #[inline]
    pub fn signed_size(&self) -> f32 {
        let to_f: f32 = self.to.into();
        let from_f: f32 = self.from.into();
        to_f - from_f
    }
}

#[derive(Clone,Copy)]
pub enum TouchEvent {
    TouchDown(Touch),
    TouchMove(Touch),
    TouchUp(Touch),
}

#[derive(Clone,Copy)]
pub struct Touch {
    pub location: lcd::Point,
    pub time: time::TickTime,
}

use self::TouchEvent::*;
impl TouchEvent {
    pub fn touch(self) -> Touch {
        match self {
            TouchDown(t) | TouchMove(t) | TouchUp(t) => t,
        }
    }
}

use stm32f7::system_clock;

#[derive(Copy,Clone)]
pub struct TickTime {
    ticks: usize,
    source: &'static ClockSource,
}

impl TickTime {
    pub fn to_msecs(&self) -> usize {
        self.source.to_msecs(self)
    }
}

#[inline]
pub fn delta(a: &TickTime, b: &TickTime) -> TickTime {
    if (a.source as *const ClockSource) != (b.source as *const ClockSource) {
        panic!("delta between tick times from different sources");
    }
    let delta_ticks = {
        if a.ticks < b.ticks {
            b.ticks - a.ticks
        } else {
            a.ticks - b.ticks
        }
    };
    return TickTime{
        ticks: delta_ticks,
        source: a.source,
    }

}

pub trait ClockSource {
    fn get_ticks(&'static self) -> TickTime;
    fn to_msecs(&'static self, ticks: &TickTime) -> usize;
}

pub struct SystemClock;

pub static SYSCLOCK: SystemClock = SystemClock{};

impl ClockSource for SystemClock {
    fn get_ticks(&'static self) -> TickTime {
        return TickTime{
            ticks: system_clock::ticks(),
            source: self,
        }
    }
    fn to_msecs(&self, tt: &TickTime) -> usize {
        return tt.ticks;
    }
}
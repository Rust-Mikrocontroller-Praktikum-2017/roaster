use model::{TimeTemp, Temperature, Time};

pub struct Ramp {
    pub start: TimeTemp,
    pub end: TimeTemp,
}

impl Ramp {
    pub fn evaluate_diff(&self, time: Time) -> f32 {
        assert!(time > self.start.time);
        if time > self.end.time {
            0f32
        } else {
            (self.end.temp - self.start.temp) / (self.end.time - self.start.time)
        }
    }

    pub fn evaluate(&self, time: Time) -> Temperature {
        assert!(time > self.start.time);
        if time > self.end.time {
            self.end.temp
        } else {
            let m = (self.end.temp - self.start.temp) / (self.end.time - self.start.time);
            let c = self.start.temp - self.start.time * m;
            m * time + c
        }
    }
}

use model::{TimeTemp, Temperature, Time};

pub fn evaluate_ramp_diff(time: Time, ramp_start: TimeTemp, ramp_end: TimeTemp) -> f32 {
    assert!(time > ramp_start.time);
    if time > ramp_end.time {
        0f32
    } else {
        (ramp_end.temp - ramp_start.temp) / (ramp_end.time - ramp_start.time)
    }
}

pub fn evaluate_ramp(time: Time, ramp_start: TimeTemp, ramp_end: TimeTemp) -> Temperature {
    assert!(time > ramp_start.time);
    if time > ramp_end.time {
        ramp_end.temp
    } else {
        let m = (ramp_end.temp - ramp_start.temp) / (ramp_end.time - ramp_start.time);
        let c = ramp_start.temp - ramp_start.time * m;
        m * time + c
    }
}
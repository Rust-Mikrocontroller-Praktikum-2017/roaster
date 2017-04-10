use time;

pub struct PIDController {
    k_p: f32,
    k_i: f32,
    k_d: f32,
    last_error: Option<f32>,
    integral: f32,
}

impl PIDController {
    pub fn new(k_p: f32, k_i: f32, k_d: f32) -> PIDController {
        PIDController {
            k_p: k_p,
            k_i: k_i,
            k_d: k_d,
            last_error: None,
            integral: 0f32,
        }
    }

    pub fn cycle(&mut self, error: f32, delta_t: &time::TickTime) -> f32 {
        let secs = delta_t.to_secs();
        let d = if let Some(last_error) = self.last_error {
            (error - last_error) / secs
        } else { 0f32 };
        self.last_error = Some(error);
        self.integral += error * secs;
        let i = self.integral;
        let p = error;
        self.k_p * p + self.k_i * i + self.k_d * d
    }

    pub fn reset(&mut self) {
        self.last_error = None;
        self.integral = 0f32;
    }
}
use model::TimeTemp;
use stm32f7::lcd::{Lcd, Color, Layer, Point, Line, Rect};
use model::{Range, Time, Temperature};
use time::{TickTime,delta};
use util;

pub struct Plot {
    last_measurement: TimeTemp,
    current_drag: Option<Drag>,
    drag_scale_delta: (f32, f32),
    x_range: Range<f32>,
    y_range: Range<f32>,
}

#[derive(Clone,Copy)]
pub struct Touch {
    pub location: Point,
    pub time: TickTime,
}

#[derive(Clone,Copy,PartialEq)]
pub enum DragDirection {
    None,
    Horizontal,
    Vertical,
}

#[derive(Clone,Copy)]
struct Drag {
    last_touch: Touch,
    direction: DragDirection,
}

const X_PX_RANGE: Range<u16> = Range{from: 20, to: 460};
const Y_PX_RANGE: Range<u16> = Range{from: 252, to: 20};

const X_PX_DRAG_RANGE: Rect = Rect{ // TODO
    origin: Point{ x: 0, y: 232},
    width: 480,
    height: 40,
};
const Y_PX_DRAG_RANGE: Rect = Rect{ // TODO
    origin: Point{ x: 0, y: 0 },
    width: 40,
    height: 272,
};

const X_TICK_DIST: f32 = 30f32;
const Y_TICK_DIST: f32 = 25f32;

impl Plot {
    pub fn new(x_range: Range<f32>, y_range: Range<f32>) -> Plot {
        Plot {
            last_measurement: TimeTemp{time: 0f32, temp: 0f32},
            x_range: x_range,
            y_range: y_range,
            current_drag: None,
            drag_scale_delta: (0.1f32,0.1f32),
        }
        // TODO assert drag_horizontal_zone and drag_vertical_zone are not overlapping
    }

    fn transform_time(&self, time: Time) -> u16 {
        ((X_PX_RANGE.from as f32) + ((X_PX_RANGE.signed_size() as f32) / self.x_range.signed_size()) * (time - self.x_range.from)) as u16
    }

    fn transform_temp(&self, temp: Temperature) -> u16 {
        ((Y_PX_RANGE.from as f32) + ((Y_PX_RANGE.signed_size() as f32) / self.y_range.signed_size()) * (temp - self.y_range.from)) as u16
    }

    pub fn transform(&self, measurement: &TimeTemp) -> Point {
        Point {
            x: self.transform_time(measurement.time),
            y: self.transform_temp(measurement.temp),
        }
    }

    pub fn draw_axis(&self, lcd: &mut Lcd) {
        let axis_color = Color::from_hex(0xffffff).to_argb1555();
        let drag_color = Color::from_hex(0x222222).to_argb1555();

        let x_axis_line = Line {
            from: Point{x: X_PX_RANGE.from, y: Y_PX_RANGE.from},
            to: Point{x: X_PX_RANGE.to, y: Y_PX_RANGE.from}
        };
        let y_axis_line = Line {
            from: Point{x: X_PX_RANGE.from, y: Y_PX_RANGE.from},
            to: Point{x: X_PX_RANGE.from, y: Y_PX_RANGE.to}
        };

        lcd.fill_rect_color(Y_PX_DRAG_RANGE, Layer::Layer1, drag_color);
        lcd.fill_rect_color(X_PX_DRAG_RANGE, Layer::Layer1, drag_color);

        lcd.draw_line_color(x_axis_line, Layer::Layer1, axis_color);
        lcd.draw_line_color(y_axis_line, Layer::Layer1, axis_color);

        let mut x_tick = self.x_range.from;
        while x_tick <= self.x_range.to {
            let x_tick_px = self.transform_time(x_tick);
            lcd.draw_line_color(Line {
                from: Point{x: x_tick_px, y: Y_PX_RANGE.from - 2},
                to: Point{x: x_tick_px, y: Y_PX_RANGE.from + 2}
            }, Layer::Layer1, axis_color);
            x_tick += X_TICK_DIST;
        }

        let mut y_tick = self.y_range.from;
        while y_tick <= self.y_range.to {
            let y_tick_px = self.transform_temp(y_tick);
            lcd.draw_line_color(Line {
                from: Point{x: X_PX_RANGE.from - 2, y: y_tick_px},
                to: Point{x: X_PX_RANGE.from + 2, y: y_tick_px}
            }, Layer::Layer1, axis_color);
            y_tick += Y_TICK_DIST;
        }
    }

    pub fn add_measurement(&mut self, measurement: TimeTemp, lcd: &mut Lcd) {
        lcd.draw_line_color(
            Line{
                from: self.transform(&self.last_measurement),
                to: self.transform(&measurement),
            }, Layer::Layer1, Color::from_hex(0xff0000).to_argb1555());
        self.last_measurement = measurement;

    }

    pub fn event_loop_touch(&mut self, touch: Touch) -> Option<(DragDirection,f32)> {

        let drag_zone = {
            if X_PX_DRAG_RANGE.contains_point(&touch.location) {
                DragDirection::Horizontal
            } else if Y_PX_DRAG_RANGE.contains_point(&touch.location) {
                DragDirection::Vertical
            } else {
                DragDirection::None
            }
        };

        // We don't care about touches not in drag zones
        if let DragDirection::None = drag_zone {
            return None;
        }

        if let None = self.current_drag {
            self.current_drag = Some(Drag{
                last_touch: touch,
                direction: drag_zone,
            });
            return None;
        }

        let mut drag = &mut self.current_drag.unwrap();

        // Timeout between touches & switch of zones
        let since_last_msecs = delta(&drag.last_touch.time, &touch.time).to_msecs();
        if since_last_msecs >= 40 || drag.direction != drag_zone {
            self.current_drag = None; // Cancel current drag
            return None;
        }

        assert!(drag.direction != DragDirection::None);

        let touch_delta = match drag.direction {
            DragDirection::Horizontal => {
                signed_delta(drag.last_touch.location.x, touch.location.x) *
                    self.drag_scale_delta.0
            },
            DragDirection::Vertical   => {
                signed_delta(drag.last_touch.location.y, touch.location.y) *
                    self.drag_scale_delta.1
            },
            _                         => 0.0f32,
        };

        drag.last_touch = touch;

        return Some((drag.direction, touch_delta));

    }

}

fn signed_delta(from: u16, to: u16) -> f32 {
    return ((to as i32) - (from as i32)) as f32;
}
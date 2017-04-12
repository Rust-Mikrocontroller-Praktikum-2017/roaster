use model::TimeTemp;
use stm32f7::lcd::*;
//{Lcd, Color, Layer, Point, Line, Rect,TextBox,Font,Alignment,CLEAR_COLOR};
use model::{Range, Time, Temperature, Touch};
use time::{TickTime,delta,SYSCLOCK,ClockSource};
use ramp::Ramp;
use util;

use collections::*;

pub struct Plot {
    last_measurement: TimeTemp,
    current_drag: Option<Drag>,
    drag_scale_delta: (f32, f32),
    x_range: Range<f32>,
    y_range: Range<f32>,
    axis_font: &'static Font<'static>,
    axis_color: Color,
    drag_zone_color: Color,
    drag_timeout: usize,
    ramp: Ramp,
    last_ramp_line: Line,
    curval_textbox: TextBox<'static>,
    last_curval_textbox_update: Option<TickTime>,
    target_textbox: TextBox<'static>,
    last_target_textbox_update: Option<TickTime>,
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

pub const X_PX_RANGE: Range<u16> = Range{from: 24, to: 460};
pub const Y_PX_RANGE: Range<u16> = Range{from: 252, to: 20};

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

const X_TICK_DIST: f32 = 60f32;
const Y_TICK_DIST: f32 = 25f32;

const CURVAL_LABEL: &'static str = "Cur";
const TARGET_LABEL: &'static str = "Tar";
const LABEL_LEN: usize = 3;


impl Plot {
    pub fn new(x_range: Range<f32>, y_range: Range<f32>, axis_font: &'static Font<'static>, target_label_font: &'static Font<'static>, axis_color: Color, drag_zone_color: Color, drag_timeout: usize) -> Plot {

        // TODO fix hardocing
        let curval_textbox = TextBox{
            alignment: Alignment::Right,
            canvas: Rect{ origin: Point{x: 480 - 170 - 6, y: 194},
                        width: 170,
                        height: target_label_font.size + 6 // TODO compute offset
                        },
            font: target_label_font,
            bg_color: CLEAR_COLOR,
            fg_color: axis_color,
        };
        let target_textbox = TextBox{
            alignment: Alignment::Right,
            canvas: Rect{origin: curval_textbox.canvas.anchor_point(Anchor::LowerLeft)
                                + Point{x: 0, y: 2}, // padding,
                        width: curval_textbox.canvas.width,
                        height: target_label_font.size + 6 // TODO compute offset
                        },
            font: target_label_font,
            bg_color: CLEAR_COLOR,
            fg_color: axis_color,
        };

        Plot {
            last_measurement: TimeTemp{time: 0f32, temp: 0f32},
            x_range: x_range,
            y_range: y_range,
            current_drag: None,
            drag_scale_delta: (0.1f32,0.1f32),
            axis_font: axis_font,
            axis_color: axis_color,
            drag_zone_color: drag_zone_color,
            drag_timeout: drag_timeout,
            target_textbox: target_textbox,
            last_target_textbox_update: None,
            curval_textbox: curval_textbox,
            last_curval_textbox_update: None,
            ramp: Ramp {
                start: TimeTemp{time: 0f32, temp: 0f32},
                end: TimeTemp{time: 60f32, temp: 100f32},
            },
            last_ramp_line: Line {
                from: Point{x: 0, y: 0},
                to: Point{x: 0, y: 0},
            }
        }
        // TODO assert drag_horizontal_zone and drag_vertical_zone are not overlapping
    }

    pub fn transform_ranges(from_range: Range<f32>, to_range: Range<u16>, value: f32) -> u16 {
        ((to_range.from as f32) + ((to_range.signed_size() as f32) / from_range.signed_size()) * (value - from_range.from)) as u16
    }

    pub fn transform_time(&self, time: Time) -> u16 {
        Plot::transform_ranges(self.x_range, X_PX_RANGE, time)
    }

    pub fn transform_temp(&self, temp: Temperature) -> u16 {
        Plot::transform_ranges(self.y_range, Y_PX_RANGE, temp)
    }

    pub fn transform(&self, measurement: &TimeTemp) -> Point {
        Point {
            x: self.transform_time(measurement.time),
            y: self.transform_temp(measurement.temp),
        }
    }

    pub fn draw_axis(&self, lcd: &mut Lcd) {

        let x_axis_line = Line {
            from: Point{x: X_PX_RANGE.from, y: Y_PX_RANGE.from},
            to: Point{x: X_PX_RANGE.to, y: Y_PX_RANGE.from}
        };
        let y_axis_line = Line {
            from: Point{x: X_PX_RANGE.from, y: Y_PX_RANGE.from},
            to: Point{x: X_PX_RANGE.from, y: Y_PX_RANGE.to}
        };

        lcd.fill_rect_color(Y_PX_DRAG_RANGE, Layer::Layer1, self.drag_zone_color.to_argb1555());
        lcd.fill_rect_color(X_PX_DRAG_RANGE, Layer::Layer1, self.drag_zone_color.to_argb1555());

        lcd.draw_line_color(x_axis_line, Layer::Layer1, self.axis_color.to_argb1555());
        lcd.draw_line_color(y_axis_line, Layer::Layer1, self.axis_color.to_argb1555());

        let mut tb = TextBox{
            canvas: Rect{origin:Point{x:0, y:0}, width: 18, height: 14},
            font: self.axis_font,
            alignment: Alignment::Center,
            bg_color: self.drag_zone_color,//Color::from_hex(0xff0000),
            fg_color: self.axis_color,
        };


        let mut x_tick = self.x_range.from;
        while x_tick <= self.x_range.to {

            let x_tick_px = self.transform_time(x_tick);

            let t_pad = 1;
            let tick_line_radius = 2;

            // V: [Y_PX_RANGE.from]-[tick_line_radius]-[t_pad]
            {
                tb.canvas.origin = Point{x: x_tick_px - tb.canvas.width/2,
                                         y: Y_PX_RANGE.from + tick_line_radius + t_pad};
                tb.redraw(format!("{}", x_tick).as_str(), |p,c| {
                    lcd.draw_point_color(p, Layer::Layer1, c.to_argb1555())
                });
            }

            lcd.draw_line_color(Line {
                from: Point{x: x_tick_px, y: Y_PX_RANGE.from - 2},
                to: Point{x: x_tick_px, y: Y_PX_RANGE.from + 2}
            }, Layer::Layer1, self.axis_color.to_argb1555());


            x_tick += X_TICK_DIST;
        }

        let mut y_tick = self.y_range.from;
        while y_tick <= self.y_range.to {

            let y_tick_px = self.transform_temp(y_tick);

            let t_pad = 2;
            let tick_line_radius = 2;

            // |-[t_pad]-[tb.canvas.width]-[t_pad]-(tick_line)
            {
                tb.alignment = Alignment::Right;
                tb.canvas.width = X_PX_RANGE.from - 2*t_pad - tick_line_radius;
                tb.canvas.origin = Point{x: X_PX_RANGE.from
                                            - tick_line_radius
                                            - t_pad
                                            - tb.canvas.width,
                                         y: y_tick_px - tb.canvas.height/2};
                tb.redraw(format!("{}", y_tick).as_str(), |p,c| {
                    lcd.draw_point_color(p, Layer::Layer1, c.to_argb1555())
                });
            }

             lcd.draw_line_color(Line {
                from: Point{x: X_PX_RANGE.from - tick_line_radius, y: y_tick_px},
                to: Point{x: X_PX_RANGE.from + tick_line_radius, y: y_tick_px}
            }, Layer::Layer1, self.axis_color.to_argb1555());

            y_tick += Y_TICK_DIST;
        }
    }

    pub fn add_measurement(&mut self, measurement: TimeTemp, lcd: &mut Lcd) {

        let ticks = SYSCLOCK.get_ticks();
        if self.last_curval_textbox_update.map_or(true, |x| delta(&x, &ticks).to_msecs() > 1000) {
            self.last_curval_textbox_update = Some(ticks);
            self.curval_textbox.redraw(rtval_format(CURVAL_LABEL, LABEL_LEN, measurement).as_str(), |p,c| {
                lcd.draw_point_color(p, Layer::Layer2, c.to_argb1555());
            });
        }

        lcd.draw_line_color(
            Line{
                from: self.transform(&self.last_measurement),
                to: self.transform(&measurement),
            }, Layer::Layer1, Color::from_hex(0xff0000).to_argb1555());
        self.last_measurement = measurement;

    }

    pub fn draw_ramp(&mut self, lcd: &mut Lcd) {
        //Clear old line
        lcd.draw_line_color(self.last_ramp_line, Layer::Layer2, Color::rgba(0, 0, 0, 0).to_argb1555());

        //Draw new line
        let p_start = self.transform(&self.ramp.start);
        let p_end = self.transform(&self.ramp.end);
        let line = Line{from: p_start, to: p_end};
        let c: u16 = Color::from_hex(0x00ff00).to_argb1555();
        lcd.draw_line_color(line, Layer::Layer2, c);
        self.last_ramp_line = line;
    }

    pub fn handle_touch(&mut self, touch: Touch, lcd: &mut Lcd) {

        let drag = self.get_drag_gesture(touch);
        if let None = drag {
            return;
        }
        let (dir, drag_delta) = drag.unwrap();

        match dir {
            DragDirection::Horizontal => self.ramp.end.time = util::clamp_range(self.ramp.end.time + drag_delta, self.x_range),
            DragDirection::Vertical   => self.ramp.end.temp = util::clamp_range(self.ramp.end.temp - drag_delta, self.y_range),
            _                         => {},
        }

        //Target has to be in the future
        if self.ramp.end.time < self.last_measurement.time + 10f32 {
            self.ramp.end.time = self.last_measurement.time + 10f32;
        }

        self.ramp.start = self.last_measurement;

        let ticks = SYSCLOCK.get_ticks();
        if self.last_target_textbox_update.map_or(true, |x| delta(&x, &ticks).to_msecs() > 1000) {
            self.last_target_textbox_update = Some(ticks);
            self.target_textbox.redraw(rtval_format(TARGET_LABEL, LABEL_LEN, self.ramp.end).as_str(), |p,c| {
                lcd.draw_point_color(p, Layer::Layer2, c.to_argb1555());
            });
        }

        self.draw_ramp(lcd);
    }

    fn get_drag_gesture(&mut self, touch: Touch) -> Option<(DragDirection,f32)> {

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
        if since_last_msecs >= self.drag_timeout || drag.direction != drag_zone {
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

    pub fn ramp(&self) -> &Ramp {
        &self.ramp
    }

}

fn signed_delta(from: u16, to: u16) -> f32 {
    return ((to as i32) - (from as i32)) as f32;
}


#[inline]
fn rtval_format(label: &str, label_len: usize, measurement: TimeTemp) -> String {
    let mins = measurement.time as usize / 60;
    let secs = measurement.time as usize % 60;
    format!("{:.*} {:>5.1}°C @ {:02}:{:02}", label_len, label, measurement.temp, mins, secs)
}
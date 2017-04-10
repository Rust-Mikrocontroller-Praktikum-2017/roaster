use model::TimeTemp;
use stm32f7::lcd;
use stm32f7::lcd::{Lcd, Color, Layer, Point, Line};
use model::{Range};

pub struct Plot {
    last_measurement: TimeTemp,
    x_range: Range<f32>,
    y_range: Range<f32>,
}

const X_PX_RANGE: Range<u16> = Range{from: 10, to: 460};
const Y_PX_RANGE: Range<u16> = Range{from: 252, to: 10};

impl Plot {
    pub fn new(x_range: Range<f32>, y_range: Range<f32>) -> Plot {
        Plot {
            last_measurement: TimeTemp{time: 0f32, temp: 0f32},
            x_range: x_range,
            y_range: y_range,
        }
    }

    fn transform(&self, measurement: &TimeTemp) -> Point {
        Point {
            x: ((X_PX_RANGE.from as f32) + ((X_PX_RANGE.signed_size() as f32) / self.x_range.signed_size()) * (measurement.time - self.x_range.from)) as u16,
            y: ((Y_PX_RANGE.from as f32) + ((Y_PX_RANGE.signed_size() as f32) / self.y_range.signed_size()) * (measurement.temp - self.y_range.from)) as u16,
        }
    }

    pub fn draw_axis(&self, lcd: &mut Lcd) {
        let axis_color = Color::from_hex(0xffffff).to_argb1555();
        let origin_y = lcd::LCD_SIZE.height - 10;

        let x_axis_line = Line {
            from: Point{x: X_PX_RANGE.from, y: Y_PX_RANGE.from},
            to: Point{x: X_PX_RANGE.to, y: Y_PX_RANGE.from}
        };
        let y_axis_line = Line {
            from: Point{x: X_PX_RANGE.from, y: Y_PX_RANGE.from},
            to: Point{x: X_PX_RANGE.from, y: Y_PX_RANGE.to}
        };

        lcd.draw_line_color(x_axis_line, Layer::Layer1, axis_color);
        lcd.draw_line_color(y_axis_line, Layer::Layer1, axis_color);
    }
    pub fn add_measurement(&mut self, measurement: TimeTemp, lcd: &mut Lcd) {
        let origin_y = lcd::LCD_SIZE.height - 10;
        lcd.draw_line_color(
            Line{
                from: self.transform(&self.last_measurement),
                to: self.transform(&measurement),
            }, Layer::Layer1, Color::from_hex(0xff0000).to_argb1555());
        self.last_measurement = measurement;

    }

}

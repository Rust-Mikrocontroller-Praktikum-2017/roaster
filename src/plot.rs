use model::TimeTemp;
use stm32f7::lcd;
use stm32f7::lcd::{Lcd, Color, Layer, Point, Line};

pub struct Plot {
    pub last_measurement: TimeTemp

}

impl Plot {
    pub fn draw_axis(&self, lcd: &mut Lcd) {
        let axis_color = Color::from_hex(0xffffff).to_argb1555();
        let origin_y = lcd::LCD_SIZE.height - 10;

        let x_axis_line = Line {
            from: Point{x: 10, y: origin_y},
            to: Point{x: lcd::LCD_SIZE.width - 10, y: origin_y}
        };
        let y_axis_line = Line {
            from: Point{x: 10, y: origin_y},
            to: Point{x: 10, y: 10}
        };

        lcd.draw_line_color(x_axis_line, Layer::Layer1, axis_color);
        lcd.draw_line_color(y_axis_line, Layer::Layer1, axis_color);
    }
    pub fn add_measurement(&mut self, measurement: TimeTemp, lcd: &mut Lcd) {
        let origin_y = lcd::LCD_SIZE.height - 10;
        lcd.draw_line_color(
            Line{
                from: Point{x: 10 + (self.last_measurement.time / 500) as u16, y: origin_y - self.last_measurement.temp / 10},
                to: Point{x: 10 + (measurement.time / 500) as u16, y: origin_y - measurement.temp / 10}
            }, Layer::Layer1, Color::from_hex(0xff0000).to_argb1555());
        self.last_measurement = measurement;

    }

}

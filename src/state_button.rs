use lcd;
use plot;
use time;

#[derive(Copy, Clone)]
pub enum State {
    RESETTED,
    RUNNING,
    STOPPED,
}

pub fn next_state(state: State) -> State {
    match state {
        State::RESETTED => State::RUNNING,
        State::RUNNING => State::STOPPED,
        State::STOPPED => State::RESETTED,
    }
}

pub struct StateButton {
    bg_color: lcd::Color,
    rect: lcd::Rect,
    last_touch_time: Option<time::TickTime>,
    state: State,
}

impl StateButton {
    pub fn new(bg_color: lcd::Color, rect: lcd::Rect) -> StateButton {
        StateButton {
            bg_color: bg_color,
            rect: rect,
            last_touch_time: None,
            state: State::RESETTED,
        }
    }

    pub fn state(&self) -> State {
        self.state
    }

    pub fn render(&self, lcd: &mut lcd::Lcd) {
        lcd.fill_rect_color(self.rect, lcd::Layer::Layer1, self.bg_color.to_argb1555());
        let center_color = match self.state {
            State::RESETTED => lcd::Color::from_hex(0xff0000),
            State::RUNNING => lcd::Color::from_hex(0x00ff00),
            State::STOPPED => lcd::Color::from_hex(0xffff00),
        };
        lcd.fill_rect_color(lcd::Rect{
            origin: lcd::Point{x: self.rect.origin.x + 5, y: self.rect.origin.y + 5},
            width: self.rect.width - 10,
            height: self.rect.height - 10,
        }, lcd::Layer::Layer1, center_color.to_argb1555())
    }

    ///Handles touches
    ///Returns Some(new_state) if state was changed, else None
    pub fn handle_touch(&mut self, touch: plot::Touch) -> Option<State> { //TODO move touch out of plot
        if self.rect.contains_point(&touch.location) {

            //Is this the first touch or has some time passed since the last touch
            if self.last_touch_time.map_or(true, |last_touch_time| time::delta(&touch.time, &last_touch_time).to_msecs() > 400) {
                self.state = next_state(self.state);
                self.last_touch_time = Some(touch.time);
                return Some(self.state);
            } else {
                self.last_touch_time = Some(touch.time);
            }
        }
        None
    }

}
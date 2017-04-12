#![no_std]
#![no_main]
#![feature(asm)]
#![feature(collections)]

extern crate stm32f7_discovery as stm32f7;
extern crate collections;

extern crate r0;

pub mod plot;
pub mod model;
pub mod temp_sensor;
pub mod time;
pub mod util;
pub mod pid;
pub mod ramp;
pub mod state_button;
mod leak;

use stm32f7::{system_clock,board,embedded,sdram,lcd,touch,i2c};
use stm32f7::lcd::*;
use embedded::interfaces::gpio;
use embedded::interfaces::gpio::{Gpio};
use board::spi::Spi;
use time::*;
use util::*;
use plot::DragDirection;
use embedded::util::delay;

use model::TouchEvent::*;

use collections::*;

use collections::boxed::Box;
use leak::Leak;

use self::temp_sensor::{TemperatureSensor,Max6675};
use state_button::State;

static TTF: &[u8] = include_bytes!("RobotoMono-Bold.ttf");

#[no_mangle]
pub unsafe extern "C" fn reset() {

    extern "C" {
        static __DATA_LOAD: u32;
        static __DATA_END: u32;
        static mut __DATA_START: u32;
        static mut __BSS_START: u32;
        static mut __BSS_END: u32;
    }
    let data_load = &__DATA_LOAD;
    let data_start = &mut __DATA_START;
    let data_end = &__DATA_END;
    let bss_start = &mut __BSS_START;
    let bss_end = &__BSS_END;

    r0::init_data(data_start, data_end, data_load);
    r0::zero_bss(bss_start, bss_end);

    stm32f7::heap::init();

    // enable floating point unit
    let scb = stm32f7::cortex_m::peripheral::scb_mut();
    scb.cpacr.modify(|v| v | 0b1111 << 20);
    asm!("DSB; ISB;"::::"volatile"); // pipeline flush

    main(board::hw());
}

                    // WORKAROUND: rust compiler will inline & reorder fp instructions into
#[inline(never)]    //             reset() before the FPU is initialized
fn main(hw: board::Hardware) -> ! {

    let board::Hardware {
        rcc,
        pwr,
        flash,
        fmc,
        ltdc,
        gpio_a,
        gpio_b,
        gpio_c,
        gpio_d,
        gpio_e,
        gpio_f,
        gpio_g,
        gpio_h,
        gpio_i,
        gpio_j,
        gpio_k,
        spi_2,
        i2c_3,
        ..
    } = hw;

    let mut gpio = Gpio::new(gpio_a,
                             gpio_b,
                             gpio_c,
                             gpio_d,
                             gpio_e,
                             gpio_f,
                             gpio_g,
                             gpio_h,
                             gpio_i,
                             gpio_j,
                             gpio_k);

    system_clock::init(rcc, pwr, flash);

    // Peripheral clock configuration
    {
        // enable all gpio ports
        rcc.ahb1enr.update(|r| {
            r.set_gpioaen(true);
            r.set_gpioben(true);
            r.set_gpiocen(true);
            r.set_gpioden(true);
            r.set_gpioeen(true);
            r.set_gpiofen(true);
            r.set_gpiogen(true);
            r.set_gpiohen(true);
            r.set_gpioien(true);
            r.set_gpiojen(true);
            r.set_gpioken(true);
        });

        // Enable SPI_2
        rcc.apb1enr.update(|apb1enr| {
            apb1enr.set_spi2en(true);
        });

        delay(1);

    }


    // i2c configuration
    i2c::init_pins_and_clocks(rcc, &mut gpio);
    let mut i2c_3 = i2c::init(i2c_3);
    i2c_3.test_1();
    i2c_3.test_2();

    let mut temp_sensor = temp_sensor_init_spi2(&mut gpio, spi_2);
    // init sdram (needed for display buffer)
    sdram::init(rcc, fmc, &mut gpio);

    let pwm_pin = (gpio::Port::PortG, gpio::Pin::Pin6);

    let mut pwm_gpio = gpio.to_output(pwm_pin,
                   gpio::OutputType::PushPull,
                   gpio::OutputSpeed::High,
                   gpio::Resistor::NoPull)
        .expect("Could not configure pwm pin");


    let axis_color = Color::from_hex(0xffffff);
    let drag_color = Color::from_hex(0x000000);
    let grid_color = Color::from_hex(0x444444);

    // lcd controller
    let mut lcd = lcd::init(ltdc, rcc, &mut gpio);
    touch::check_family_id(&mut i2c_3).unwrap();

    loop {
    SYSCLOCK.reset();
    lcd.clear_screen();

    lcd.set_background_color(Color::from_hex(0x000000));

    let plot_font = Box::new(Font::new(TTF, 11).unwrap()).leak();
    let rtval_font = Box::new(Font::new(TTF, 14).unwrap()).leak();

    let mut plot = plot::Plot::new(model::Range::new(0f32, (20*60) as f32),
                                   model::Range::new(0f32, 200f32),
                                   plot_font,
                                   rtval_font,
                                   axis_color,
                                   grid_color,
                                   drag_color,
                                   70, // drag timeout
                    );

    plot.draw_axis(&mut lcd);

    //let mut pid_controller = pid::PIDController::new(0.3f32, 0.0f32, 0.0f32);
    //let mut pid_controller = pid::PIDController::new(0.1f32, 0.0f32, 0.3f32); // Definitely better than first
    let mut pid_controller = pid::PIDController::new(0.2f32, 0.0f32, 0.3f32);

    let mut smoother = pid::Smoother::new(10);

    let mut measurement_start_system_time = SYSCLOCK.get_ticks();
    let mut last_measurement_system_time = SYSCLOCK.get_ticks();

    let mut duty_cycle: usize = 0;

    let mut temp = 20f32;

    let mut state_button = state_button::StateButton::new(
        Color::from_hex(0x222222),
        Rect{origin: Point{x: 440, y: 0}, width: 40, height: 40}
    );
    state_button.render(&mut lcd);

    let mut last_touch_event = None;

    'mainloop: loop {

        let ticks = SYSCLOCK.get_ticks();

        let delta_measurement = time::delta_checked(&last_measurement_system_time, &ticks);

        if delta_measurement.to_msecs() >= 500 {
            let val = temp;//temp_sensor.read();
            let measurement_time = time::delta_checked(&measurement_start_system_time, &ticks).to_secs();
            let measurement = model::TimeTemp{
                time: measurement_time, // TODO just integer divide here?
                temp: val as f32,
            };
            match state_button.state() {
                State::RUNNING => plot.add_measurement(measurement, &mut lcd),
                State::RESETTED => {
                    plot.set_measurement(model::TimeTemp{time: 0f32, temp: measurement.temp}, &mut lcd);
                    plot.update_ramp_start(&mut lcd);
                },
                State::STOPPED => {},
            }

            if let State::RUNNING = state_button.state() {
                smoother.push_value(val);
                let smooth_temp = smoother.get_average();

                let ramp_target_temp = plot.ramp().evaluate(measurement_time);

                let error = ramp_target_temp - smooth_temp;
                let pid_value = pid_controller.cycle(error, &delta_measurement);
                duty_cycle = (util::clamp(pid_value, 0f32, 1f32) * 1000f32) as usize;

                lcd.draw_point_color(
                    Point{
                        x: plot.transform_time(measurement_time),
                        y: plot::Plot::transform_ranges(model::Range{from: 0f32, to: 1f32}, plot::Y_PX_RANGE, pid_value)
                    }, Layer::Layer2, Color::from_hex(0x0000ff).to_argb1555());

                let pid_clamped = util::clamp(pid_value, 0f32, 1f32);
                temp += (pid_clamped - 0.3) * delta_measurement.to_secs() * 1.0;
            } else {
                duty_cycle = 0;
            }
            last_measurement_system_time = ticks;
        }

        pwm_gpio.set(ticks.to_msecs() % 1000 < duty_cycle);

        // poll for new touch data
        let mut touches = false;
        for touch in &touch::touches(&mut i2c_3).unwrap() {

            touches = true;

            let touch = model::Touch{
                location: Point{
                    x: touch.x,
                    y: touch.y
                },
                time: ticks
            };

            let touch_event = match last_touch_event {
                Some(TouchDown(_)) | Some(TouchMove(_)) => TouchMove(touch),
                None | Some(TouchUp(_)) => TouchDown(touch),
            };

            match state_button.state() {
                State::RUNNING | State::RESETTED =>
                    plot.handle_touch(touch_event, &mut lcd),
                _ => {},
            }

            if let Some(new_state) = state_button.handle_touch(touch_event, &mut lcd) {
                match new_state {
                    State::RESETTED => {
                        break 'mainloop;
                    },
                    State::RUNNING => {
                        measurement_start_system_time = SYSCLOCK.get_ticks();
                        last_measurement_system_time = measurement_start_system_time;
                    },
                    _ => {},
                }
            }

            last_touch_event = Some(touch_event);

        }

        // Deliver touch-up events
        if !touches && last_touch_event.is_some() {

            let touch_event = match last_touch_event.unwrap() {
                TouchDown(t) | TouchMove(t) if time::delta(&ticks,&t.time).to_msecs() > 200 => {
                    state_button.handle_touch(TouchUp(t), &mut lcd);
                    plot.handle_touch(TouchUp(t), &mut lcd);
                    None
                },
                x => Some(x),
            };

            last_touch_event = touch_event;
        }

    }

    }


}


/// Initialize temperature sensor on SPI_2 port (GPIO pins)
/// IMPORTANT: "Table 3. Arduino connectors" in the discovery board datasheet
//             states SPI2_NSS is pin D10. This is wrong.AsMut
//             SPI2_NSS is D5, as seen in "Figure 25: Arduino Uno connectors"
fn temp_sensor_init_spi2(gpio: &mut Gpio, spi_2: &'static mut Spi) -> Max6675 {

    let sck_pin = (gpio::Port::PortI, gpio::Pin::Pin1);
    let miso_pin = (gpio::Port::PortB, gpio::Pin::Pin14);
    let mosi_pin = (gpio::Port::PortB, gpio::Pin::Pin15);
    let nss_pin = (gpio::Port::PortI, gpio::Pin::Pin0);

    gpio.to_alternate_function(sck_pin,
                               gpio::AlternateFunction::AF5,
                               gpio::OutputType::PushPull,
                               gpio::OutputSpeed::High,
                               gpio::Resistor::NoPull)
        .expect("Could not configure sck");

    // TODO the MOSI pin is not necessarily necessary for MAX6675
    gpio.to_alternate_function(mosi_pin,
                               gpio::AlternateFunction::AF5,
                               gpio::OutputType::PushPull,
                               gpio::OutputSpeed::High,
                               gpio::Resistor::NoPull)
        .expect("Could not configure mosi");

    gpio.to_alternate_function(miso_pin,
                               gpio::AlternateFunction::AF5,
                               gpio::OutputType::PushPull,
                               gpio::OutputSpeed::High,
                               gpio::Resistor::NoPull)
        .expect("Could not configure MISO pin");

    gpio.to_alternate_function(nss_pin,
                               gpio::AlternateFunction::AF5,
                               gpio::OutputType::PushPull,
                               gpio::OutputSpeed::High,
                               gpio::Resistor::NoPull)
        .expect("Could not configure NSS ");

    return Max6675::init(spi_2);

}

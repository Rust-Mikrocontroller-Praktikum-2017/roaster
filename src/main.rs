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

use collections::*;

use collections::boxed::Box;
use leak::Leak;

use self::temp_sensor::{TemperatureSensor,Max6675};

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
    let drag_color = Color::from_hex(0x222222);
    let clear_color = Color::rgba(0,0,0,0);

    // lcd controller
    let mut lcd = lcd::init(ltdc, rcc, &mut gpio);
    lcd.clear_screen();

    lcd.set_background_color(Color::from_hex(0x000000));

    let plot_font = Box::new(Font::new(TTF, 11).unwrap()).leak();

    let mut plot = plot::Plot::new(model::Range::new(0f32, (5*60) as f32),
                                   model::Range::new(0f32, 100f32),
                                   plot_font,
                                   axis_color,
                                   drag_color,
                                   70, // drag timeout
                    );

    let rtval_font = Box::new(Font::new(TTF, 14).unwrap()).leak();
    let curval_textbox = TextBox{
        alignment: Alignment::Right,
        canvas: Rect{ origin: Point{x: 480 - 170 - 6, y: 194},
                      width: 170, height: rtval_font.size + 6},
        font: rtval_font,
        bg_color: clear_color,
        fg_color: axis_color,
    };
    let target_textbox = TextBox{
        alignment: Alignment::Right,
        canvas: Rect{origin: curval_textbox.canvas.anchor_point(Anchor::LowerLeft)
                            + Point{x: 0, y: 2}, // padding,
                     width: curval_textbox.canvas.width, height: rtval_font.size + 6},
        font: rtval_font,
        bg_color: clear_color,
        fg_color: axis_color,
    };

    plot.draw_axis(&mut lcd);

    touch::check_family_id(&mut i2c_3).unwrap();

    let mut pid_controller = pid::PIDController::new(1f32, 0.01f32, 0.0f32);

    let mut last_measurement_time = SYSCLOCK.get_ticks();
    let mut last_measurement = model::TimeTemp{time: 0f32, temp: 0f32};

    let mut target = model::TimeTemp{time: 10.0f32, temp: 30.0f32};
    let mut ramp_start = model::TimeTemp{time: 0f32, temp: 0f32};
    let mut last_line = lcd::Line{from: Point{x:0, y:0}, to: Point{x:0, y:0}};

    let mut duty_cycle: usize = 0;

    let mut temp = 20f32;

    let mut last_curval_textbox_update = SYSCLOCK.get_ticks();
    let mut last_target_textbox_update = SYSCLOCK.get_ticks();


    loop {

        let ticks = SYSCLOCK.get_ticks();

        let delta_measurement = time::delta(&ticks, &last_measurement_time);

        if delta_measurement.to_msecs() >= 500 {
            let val = temp;//temp_sensor.read();
            let measurement = model::TimeTemp{
                time: ticks.to_secs(), // TODO just integer divide here?
                temp: val as f32,
            };
            plot.add_measurement(measurement, &mut lcd);


            let ramp_target_temp = ramp::evaluate_ramp(ticks.to_secs(), ramp_start, target);

            let error = ramp_target_temp - measurement.temp;
            let pid_value = pid_controller.cycle(error, &delta_measurement);
            duty_cycle = if pid_value < 0f32 { 0 } else if pid_value > 1f32 { 1000 } else {(pid_value * 1000f32) as usize};
            lcd.draw_point_color(plot.transform(&model::TimeTemp{time: ticks.to_secs(), temp: pid_value * 100f32}), Layer::Layer2, Color::from_hex(0x0000ff).to_argb1555());
            let pid_clamped = if pid_value < 0f32 { 0f32 } else if pid_value > 1f32 { 1f32 } else {pid_value};
            temp += (pid_clamped - 0.3) * delta_measurement.to_secs() * 1.0;
            last_measurement_time = ticks;
            last_measurement = measurement;
        }

        pwm_gpio.set(ticks.to_msecs() % 1000 < duty_cycle);

        // poll for new touch data

        let mut processed_touches = false;
        for touch in &touch::touches(&mut i2c_3).unwrap() {

            processed_touches = true;

            let touch = plot::Touch{
                location: Point{
                    x: touch.x,
                    y: touch.y
                },
                time: ticks
            };

            match plot.event_loop_touch(touch) {
                Some((dir, delta)) => {
                    //Clear old line
                    lcd.draw_line_color(last_line, Layer::Layer2, Color::rgba(0, 0, 0, 0).to_argb1555());

                    // TODO move target
                    match dir {
                        DragDirection::Horizontal => target.time += delta,
                        DragDirection::Vertical   => target.temp -= delta,
                        _                         => {},
                    }

                    //Target has to be in the future
                    if target.time < last_measurement.time + 10f32 {
                        target.time = last_measurement.time + 10f32;
                    }

                    ramp_start = last_measurement;

                    //Draw new line
                    let p_start = plot.transform(&ramp_start);
                    let p_end = plot.transform(&target);
                    let line = Line{from: p_start, to: p_end};
                    let c: u16 = Color::from_hex(0x00ff00).to_argb1555();
                    lcd.draw_line_color(line, Layer::Layer2, c);
                    last_line = line;
                },
                _ => (),
            }


        }

        const CURVAL_LABEL: &'static str = "Cur";
        const TARGET_LABEL: &'static str = "Tar";
        const LABEL_LEN: usize = 3;

        let touch_drew_over_curval_textbox_needs_redraw =
            processed_touches &&
            time::delta(&last_curval_textbox_update, &ticks).to_msecs() > 1000/10;

        if time::delta(&last_curval_textbox_update, &ticks).to_msecs() > 1000 {
            last_curval_textbox_update = ticks;
            curval_textbox.redraw(rtval_format(CURVAL_LABEL, LABEL_LEN, last_measurement).as_str(), |p,c| {
                lcd.draw_point_color(p, Layer::Layer2, c.to_argb1555());
            });
        }

        if processed_touches &&
           time::delta(&last_target_textbox_update, &ticks).to_msecs() > 1000/10  {
            last_target_textbox_update = ticks;
            target_textbox.redraw(rtval_format(TARGET_LABEL, LABEL_LEN, target).as_str(), |p,c| {
                lcd.draw_point_color(p, Layer::Layer2, c.to_argb1555());
            });
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

    gpio.to_alternate_function(mosi_pin,
                               gpio::AlternateFunction::AF5,
                               gpio::OutputType::PushPull,
                               gpio::OutputSpeed::High,
                               gpio::Resistor::NoPull)
        .expect("Could not configure mosi");

    // TODO the MISO pin is not necessarily necessary for MAX6675
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

#[inline]
fn rtval_format(label: &str, label_len: usize, measurement: model::TimeTemp) -> String {
    let mins = measurement.time as usize / 60;
    let secs = measurement.time as usize % 60;
    format!("{:.*} {:>5.1}°C @ {:02}:{:02}", label_len, label, measurement.temp, mins, secs)
}
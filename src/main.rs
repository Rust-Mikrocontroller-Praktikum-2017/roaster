#![no_std]
#![no_main]

extern crate stm32f7_discovery as stm32f7;

extern crate r0;

pub mod plot;
pub mod model;
pub mod temp_sensor;
pub mod time;
pub mod util;

use stm32f7::{system_clock,board,embedded,sdram,lcd,touch,i2c};
use stm32f7::lcd::*;
use embedded::interfaces::gpio;
use embedded::interfaces::gpio::{Gpio};
use board::spi::Spi;
use time::*;
use util::*;
use plot::DragDirection;
use embedded::util::delay;


use self::temp_sensor::{TemperatureSensor,Max6675};

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

    let scb = stm32f7::cortex_m::peripheral::scb_mut();
    scb.cpacr.modify(|v| v | 0b1111 << 20);
    main(board::hw());
}

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

    // enable floating point unit
    unsafe {
        let scb = stm32f7::cortex_m::peripheral::scb_mut();
        scb.cpacr.modify(|v| v | 0b1111 << 20);
    }
    delay(100);

    let mut temp_sensor = temp_sensor_init_spi2(&mut gpio, spi_2);

    // init sdram (needed for display buffer)
    sdram::init(rcc, fmc, &mut gpio);

    // lcd controller
    let mut lcd = lcd::init(ltdc, rcc, &mut gpio);
    lcd.clear_screen();

    lcd.set_background_color(Color::from_hex(0x000000));

    let mut plot = plot::Plot::new(model::Range::new(0f32, (5*60) as f32), model::Range::new(0f32, 100f32));

    plot.draw_axis(&mut lcd);

    touch::check_family_id(&mut i2c_3).unwrap();

    let mut last_measurement = SYSCLOCK.get_ticks();

    let mut target = model::TimeTemp{time: 10.0f32, temp: 30.0f32};

    loop {

        let ticks = SYSCLOCK.get_ticks();

        let delta_measurement = time::delta(&ticks, &last_measurement);
        let delta_measurement = delta_measurement.to_msecs();

        if delta_measurement >= 500 {
            last_measurement = ticks;
            let val = temp_sensor.read();
            let measurement = model::TimeTemp{
                time: (delta_measurement as f32) / 1000f32, // TODO just integer divide here?
                temp: val as f32,
            };
            plot.add_measurement(measurement, &mut lcd);
        }

        // poll for new touch data

        for touch in &touch::touches(&mut i2c_3).unwrap() {

            let touch = plot::Touch{
                location: Point{
                    x: touch.x,
                    y: touch.y
                },
                time: ticks
            };

            match plot.event_loop_touch(touch) {
                Some((dir, delta)) => {
                    // TODO move target
                    match dir {
                        DragDirection::Horizontal => target.time += delta,
                        DragDirection::Vertical   => target.temp -= delta,
                        _                         => {},
                    }
                    let p = plot.transform(&target);
                    let c: u16 = Color::from_hex(0x00ff00).to_argb1555();
                    lcd.draw_point_color(p, Layer::Layer2, c);
                },
                _ => (),
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



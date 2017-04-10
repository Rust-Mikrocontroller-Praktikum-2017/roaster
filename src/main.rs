#![no_std]
#![no_main]

extern crate stm32f7_discovery as stm32f7;

extern crate r0;

pub mod plot;
pub mod model;
pub mod temp_sensor;

use stm32f7::{system_clock,board,embedded,sdram,lcd};
use stm32f7::lcd::*;
use embedded::interfaces::gpio;
use embedded::interfaces::gpio::{Gpio};
use board::spi::Spi;

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

        use embedded::util::delay;
        delay(1);

    }

    let mut temp_sensor = temp_sensor_init_spi2(&mut gpio, spi_2);

    // init sdram (needed for display buffer)
    sdram::init(rcc, fmc, &mut gpio);

    // lcd controller
    let mut lcd = lcd::init(ltdc, rcc, &mut gpio);
    lcd.clear_screen();

    lcd.set_background_color(Color::from_hex(0x000000));

    let mut plot = plot::Plot::new(model::Range::new(0f32, (5*60*1000) as f32), model::Range::new(0f32, 4096f32));

    plot.draw_axis(&mut lcd);

    let mut last_led_toggle = system_clock::ticks();
    loop {
        
        let ticks = system_clock::ticks();

        // every 0.5 seconds
        if ticks - last_led_toggle >= 500 {
            last_led_toggle = ticks;
            let val = temp_sensor.read();
            let measurement = model::TimeTemp{
                time: ticks as f32,
                temp: val as f32,
            };
            plot.add_measurement(measurement, &mut lcd);
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



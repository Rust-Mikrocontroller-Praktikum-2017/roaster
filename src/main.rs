#![no_std]
#![no_main]

extern crate stm32f7_discovery as stm32f7;

extern crate r0;

pub mod plot;
pub mod model;

use stm32f7::{system_clock,board,embedded,sdram,lcd};
use stm32f7::lcd::*;
use embedded::interfaces::gpio;
use embedded::interfaces::gpio::{Gpio};
use board::spi::Spi;
use board::rcc::Rcc;

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

    spi_init(&mut gpio, spi_2, rcc);

    // init sdram (needed for display buffer)
    sdram::init(rcc, fmc, &mut gpio);

    // lcd controller
    let mut lcd = lcd::init(ltdc, rcc, &mut gpio);
    lcd.clear_screen();

    lcd.set_background_color(Color::from_hex(0x000000));

    let mut plot = plot::Plot{last_measurement: model::TimeTemp{time: 0, temp: 0}};

    plot.draw_axis(&mut lcd);

    let mut last_led_toggle = system_clock::ticks();
    loop {
        
        let ticks = system_clock::ticks();

        // every 0.5 seconds
        if ticks - last_led_toggle >= 500 {
            last_led_toggle = ticks;
            let val = spi_read(spi_2);
            let measurement = model::TimeTemp{
                time: ticks,
                temp: val
            };
            plot.add_measurement(measurement, &mut lcd);
        }

    }

}

fn spi_init(gpio: &mut Gpio, spi_2: &mut Spi, rcc: &mut Rcc) -> () {
    //SPI Init
    
    rcc.apb1enr.update(|apb1enr| {
        apb1enr.set_spi2en(true);
    });

    use embedded::util::delay;
    delay(1);

    let sck_pin = (gpio::Port::PortI, gpio::Pin::Pin1);
    let miso_pin = (gpio::Port::PortB, gpio::Pin::Pin14);
    let mosi_pin = (gpio::Port::PortB, gpio::Pin::Pin15);
    let nss_pin = (gpio::Port::PortI, gpio::Pin::Pin0);

    let sck = gpio.to_alternate_function(sck_pin,
                                         gpio::AlternateFunction::AF5,
                                         gpio::OutputType::PushPull,
                                         gpio::OutputSpeed::High,
                                         gpio::Resistor::NoPull)
        .expect("Could not configure sck");
    let mosi = gpio.to_alternate_function(mosi_pin,
                                         gpio::AlternateFunction::AF5,
                                         gpio::OutputType::PushPull,
                                         gpio::OutputSpeed::High,
                                         gpio::Resistor::NoPull)
        .expect("Could not configure mosi");
    /*let miso = gpio.to_input(miso_pin,
                            gpio::Resistor::NoPull)
        .expect("Could not configure miso");*/
    let miso = gpio.to_alternate_function(miso_pin,
                                         gpio::AlternateFunction::AF5,
                                         gpio::OutputType::PushPull,
                                         gpio::OutputSpeed::High,
                                         gpio::Resistor::NoPull);

    let my_nss_pin = (gpio::Port::PortI, gpio::Pin::Pin2);
    let nss = gpio.to_alternate_function(nss_pin,
                                        gpio::AlternateFunction::AF5,
                                        gpio::OutputType::PushPull,
                                        gpio::OutputSpeed::High,
                                        gpio::Resistor::NoPull)
        .expect("Could not configure nss");
        
/*let mut nss = gpio.to_output(my_nss_pin,
                              gpio::OutputType::PushPull,
                              gpio::OutputSpeed::High,
                              gpio::Resistor::NoPull)
        .expect("Could not configure nss");*/

    spi_2.cr1.update(|cr1| {
        cr1.set_br(0b111);
        cr1.set_cpol(false); // clock low when inactive
        cr1.set_cpha(false); // receive on first clock transition
        cr1.set_rxonly(false);
        cr1.set_lsbfirst(false);
        cr1.set_crcen(false);
        cr1.set_mstr(true); // we are master
        cr1.set_ssm(false); // software managed ss
    });
     
    spi_2.cr2.update(|cr2| {
        cr2.set_ds(0b1111); // 16 bit data size
        cr2.set_ssoe(true); // enable ss
        cr2.set_frf(false); // frame format motorola
        cr2.set_nssp(true); // nss pulse mode
        cr2.set_frxth(false); // RXNE event is set when rx fifo contians 16 bit
        cr2.set_txdmaen(false); // no dma
        cr2.set_rxdmaen(false); // no dma
    });

    spi_2.cr1.update(|cr1| {
        cr1.set_spe(true); // SPI enable
        //cr1.set_ssi(true);
    });
}

fn spi_read(spi_2: &mut Spi) -> u16 {
    spi_2.dr.update(|dr| {
        dr.set_dr(0);   
    });

    while !spi_2.sr.read().rxne() {}

    let data:u16 = spi_2.dr.read().dr();
    return data;
}

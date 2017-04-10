extern crate stm32f7_discovery as stm32f7;

use board::spi::Spi;

use super::model::Temperature;

pub trait TemperatureSensor {
    fn read(&mut self) -> Temperature;
}

pub struct Max6675 {
    spi: &'static mut Spi,
}

/// # Abstraction for MAX6675 SPI-comptabile K-type thermocouple ADC
///
/// MAX6675 needs pause between readings to to the ADC
/// -> RXONLY mode with NSS pulse doesn't work
///
/// We use BIMODE, with MOSI not connected
/// -> When reading n bits from the data register (dr),
///    slave-select is driven active and n clock cycles are generated
impl Max6675 {

    pub fn init(spi: &'static mut Spi) -> Max6675 {

        spi.cr1.update(|cr1| {
            cr1.set_br(0b111);
            cr1.set_cpol(false);    // clock low when inactive
            cr1.set_cpha(false);    // receive on first clock transition
            cr1.set_rxonly(false);  // see struct documentation
            cr1.set_lsbfirst(false);// MSB first
            cr1.set_crcen(false);   // no CRC
            cr1.set_mstr(true);     // we are master
            cr1.set_ssm(false);     // hardware managed slave-select, see struct documentation
        });

        spi.cr2.update(|cr2| {
            cr2.set_ds(0b1111);     // 16 bit data frame size
            cr2.set_ssoe(true);     // enable slave-select line
            cr2.set_frf(false);     // frame format: motorola
            cr2.set_nssp(true);     // nss pulse mode TODO necessart??
            cr2.set_frxth(false);   // RXNE (receive fifo not empty) event is set when rx fifo is half-full, i.e. contains 16bit (one data frame)
            cr2.set_txdmaen(false); // no dma, we read from register
            cr2.set_rxdmaen(false); // no dma, we read from register
        });

        spi.cr1.update(|cr1| {
            cr1.set_spe(true);      // enable SPI enable
        });

        return Max6675 {
            spi: spi,
        }

    }
}

impl TemperatureSensor for Max6675 {

    fn read(&mut self) -> Temperature {

        self.spi.dr.update(|dr| {
            dr.set_dr(0);
        });

        // Wait until there is a value in the FIFO
        while !self.spi.sr.read().rxne() {}

        let t: u16 = self.spi.dr.read().dr();

        // FIFO should be empty after we read the data item.
        // (We are the only reader & only writer)
        assert!(!self.spi.sr.read().rxne());

        let celsius = ((t >> 3) as f32) / 4f32;

        return celsius;
    }

}
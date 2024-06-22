// Functions for controlling a TFT display via the RA8875
// Reference https://github.com/adafruit/Adafruit_RA8875/tree/master

use rp_pico as bsp;

use bsp::hal::spi;
use embedded_hal::digital::OutputPin;
use embedded_hal::spi::SpiBus;

// Needed for spi.write()
use cortex_m::prelude::*;

const LCD_WIDTH: i32 = 800;
const LCD_HEIGH: i32 = 480;

// Colors (RGB565)
///< Black Color
const RA8875_BLACK: u16 = 0x0000;
const RA8875_BLUE: u16 = 0x001F;
const RA8875_RED: u16 = 0xF800;
const RA8875_GREEN: u16 = 0x07E0;
const RA8875_CYAN: u16 = 0x07FF;
const RA8875_MAGENTA: u16 = 0xF81F;
const RA8875_YELLOW: u16 = 0xFFE0;
const RA8875_WHITE: u16 = 0xFFFF;

// Command/Data pins for SPI
const RA8875_DATAWRITE: u8 = 0x00;
const RA8875_DATAREAD: u8 = 0x40;
const RA8875_CMDWRITE: u8 = 0x80;
const RA8875_CMDREAD: u8 = 0xC0;

// Registers & bits
const RA8875_PWRR: u8 = 0x01;
const RA8875_PWRR_DISPON: u8 = 0x80;
const RA8875_PWRR_DISPOFF: u8 = 0x00;
const RA8875_PWRR_SLEEP: u8 = 0x02;
const RA8875_PWRR_NORMAL: u8 = 0x00;
const RA8875_PWRR_SOFTRESET: u8 = 0x01;

// pub struct RA8875<S: spi::State, D: spi::SpiDevice, P: spi::ValidSpiPinout<D>, const DS: u8> {
//     spi_controller: spi::Spi<S, D, P, DS>,
// }

// pub struct RA8875<SPI, CS>
// where
//     SPI: _embedded_hal_blocking_spi_Transfer<u8> + _embedded_hal_blocking_spi_Write<u8>,
//     <SPI as _embedded_hal_blocking_spi_Transfer<u8>>::Error: core::fmt::Debug,
//     <SPI as _embedded_hal_blocking_spi_Write<u8>>::Error: core::fmt::Debug,
//     CS: OutputPin,
// {
//     spi_controller: SPI,
//     cs_pin: CS,
// }

pub struct RA8875<SPI, CS>
where
    SPI: SpiBus,
    CS: OutputPin,
{
    spi_controller: SPI,
    cs_pin: CS,
}

// impl<S: spi::State, D: spi::SpiDevice, P: spi::ValidSpiPinout<D>, const DS: u8>
//     RA8875<S, D, P, DS>
impl<SPI, CS> RA8875<SPI, CS>
where
    SPI: SpiBus,
    CS: OutputPin,
{
    pub fn new(spi: SPI, cs: CS) -> Self {
        Self {
            spi_controller: spi,
            cs_pin: cs,
        }
    }

    // Read from a register
    pub fn read_reg(&mut self, reg: u8) -> u8 {
        self.write_command(reg);
        self.read_data()
    }

    // Write a command to the current register
    pub fn write_command(&mut self, d: u8) {
        // set CS pin low
        self.cs_pin.set_low().unwrap();

        // Transfer the write command identifier, followed by the specified command
        let _ = self.spi_controller.write(&[RA8875_CMDWRITE, d]);

        // set CS pin high
        self.cs_pin.set_high().unwrap();
    }

    // Read the data from the current register
    pub fn read_data(&mut self) -> u8 {
        // set CS pin low
        self.cs_pin.set_low().unwrap();

        let _ = self.spi_controller.write(&[RA8875_DATAREAD]);

        let mut buf: [u8; 1] = [0x0];
        let _ = self.spi_controller.transfer_in_place(&mut buf);

        // set CS pin high
        self.cs_pin.set_high().unwrap();

        buf[0]
    }
}

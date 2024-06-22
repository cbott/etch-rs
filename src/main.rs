//! Blinks the LED on a Pico board
#![no_std]
#![no_main]

use bsp::entry;
use defmt::*;
use defmt_rtt as _;
use embedded_hal::digital::InputPin;
use embedded_hal::digital::OutputPin;
use panic_probe as _;

use ra8875::RA8875;
// Provide an alias for our BSP so we can switch targets quickly.
use rp_pico as bsp;

use bsp::hal;
use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    pac,
    sio::Sio,
    watchdog::Watchdog,
};

// Needed for specifying SPI rates with MHz function
use hal::fugit::RateExtU32;

// Needed for spi.write()
use cortex_m::prelude::*;

mod ra8875;

#[entry]
fn main() -> ! {
    info!("Program start");
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    // External high-speed crystal on the pico board is 12Mhz
    let external_xtal_freq_hz = 12_000_000u32;
    let clocks = init_clocks_and_plls(
        external_xtal_freq_hz,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let mut led_pin = pins.led.into_push_pull_output();
    let mut a_pin = pins.gpio14.into_pull_up_input();

    // Blink 5 times
    for _ in 1..6 {
        if a_pin.is_low().unwrap() {
            info!("off!");
            led_pin.set_low().unwrap();
        } else {
            info!("on!");
            led_pin.set_high().unwrap();
        }
        delay.delay_ms(1);
    }

    // Set up SPI
    let spi_mosi = pins.gpio19.into_function::<hal::gpio::FunctionSpi>();
    let spi_miso = pins.gpio16.into_function::<hal::gpio::FunctionSpi>();
    let spi_sclk = pins.gpio18.into_function::<hal::gpio::FunctionSpi>();
    let mut cs_pin = pins.gpio10.into_push_pull_output();
    let mut rst_pin = pins.gpio9.into_push_pull_output();
    let spi = hal::spi::Spi::<_, _, _, 8>::new(pac.SPI0, (spi_mosi, spi_miso, spi_sclk));

    // Equivalent of begin() method
    cs_pin.set_high().unwrap();
    rst_pin.set_low().unwrap();
    delay.delay_ms(100);
    rst_pin.set_high().unwrap();
    delay.delay_ms(100);

    // Exchange the uninitialised SPI driver for an initialised one
    let spi = spi.init(
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
        16.MHz(),
        embedded_hal::spi::MODE_0,
    );

    let mut tft = RA8875::new(spi, cs_pin);

    let x: u8 = tft.read_reg(0);

    // Verify we're talking to the RA8875
    if x != 0x75 {
        loop {
            // infinite error blink
            led_pin.set_low().unwrap();
            delay.delay_ms(100);
            led_pin.set_high().unwrap();
            delay.delay_ms(100);
        }
    }

    loop {
        cortex_m::asm::wfi();
    }
}

// End of file

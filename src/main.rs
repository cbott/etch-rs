//! Run the etch-rs hardware
#![no_std]
#![no_main]

use bsp::entry;
use defmt::*;
use defmt_rtt as _;
use embedded_hal::digital::InputPin;
use embedded_hal::digital::OutputPin;
use panic_probe as _;

use ra8875::LCD_HEIGHT;
use ra8875::LCD_WIDTH;
// Provide an alias for our BSP so we can switch targets quickly.
use rp_pico as bsp;

use bsp::hal::{
    self,
    clocks::{init_clocks_and_plls, Clock},
    pac,
    sio::Sio,
    watchdog::Watchdog,
};

// Needed for specifying SPI rates with MHz function
use hal::fugit::RateExtU32;

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

    if a_pin.is_low().unwrap() {
        info!("off!");
        led_pin.set_low().unwrap();
    } else {
        info!("on!");
        led_pin.set_high().unwrap();
    }
    delay.delay_ms(1);

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
        2.MHz(),
        embedded_hal::spi::MODE_0,
    );

    let mut tft = ra8875::RA8875::new(spi, cs_pin);

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

    tft.initialize();
    // initialize requires a delay after, should ideally enforce this but ok for now
    delay.delay_ms(500);
    // done with begin() function

    tft.display_on(true);
    // Enable TFT - display enable tied to GPIOX
    tft.gpio_x(true);
    // PWM output for backlight
    tft.pwm1_config(true, ra8875::RA8875_PWM_CLK_DIV1024);
    tft.pwm1_out(255);

    // With hardware accelleration this is instant
    tft.fill_screen(ra8875::RA8875_WHITE);
    led_pin.set_high().unwrap();
    delay.delay_ms(1000);

    tft.fill_screen(0xAD6F);
    led_pin.set_low().unwrap();
    delay.delay_ms(1000);

    let mut i: i16 = 0;
    let mut j: i16 = 5;

    loop {
        tft.draw_pixel(
            i % ra8875::LCD_WIDTH,
            (i + j) % LCD_HEIGHT,
            ra8875::RA8875_BLACK,
        );
        i += 1;
        if i > LCD_WIDTH {
            i -= LCD_WIDTH;
            j += 5;
        }
        delay.delay_ms(10)
    }
}

//! Run the etch-rs hardware
#![no_std]
#![no_main]

use cortex_m::peripheral::mpu;
use rp_pico::hal::fugit::HertzU32;
// Provide an alias for our BSP so we can switch targets quickly.
use rp_pico as bsp;

// TODO: organize and clean up imports
use bsp::entry;
use core::cell::RefCell;
use critical_section::Mutex;
use defmt::*;
use defmt_rtt as _;
use embedded_hal::digital::InputPin;
use embedded_hal::digital::OutputPin;
use panic_probe as _;

use bsp::hal::{
    self,
    clocks::{init_clocks_and_plls, Clock},
    gpio, pac,
    sio::Sio,
    watchdog::Watchdog,
};

// Needed for specifying SPI rates with MHz function
use hal::fugit::RateExtU32;

// Interrupt macro
use bsp::hal::pac::interrupt;
use core::ops::DerefMut;

mod encoder;
mod ra8875;
use mpu_9250;

// Pin Map
// GPIO | Function
//   04 | I2C SDA
//   05 | I2C SCL
//   07 | X encoder pin A
//   08 | X encoder pin B
//   09 | RA8875 reset
//   10 | RA8875 chip select
//   11 | External LED
//   12 |
//   13 | Y encoder pin A
//   14 | Y encoder pin B
//   15 |
//   16 | SPI MISO
//   17 |
//   18 | SPI CLK
//   19 | SPI MOSI

// Pin types quickly become very long!
// We'll create some type aliases using `type` to help with that
type LedPin = gpio::Pin<gpio::bank0::Gpio11, gpio::FunctionSioOutput, gpio::PullNone>;
type EncoderX = encoder::Encoder<gpio::bank0::Gpio7, gpio::bank0::Gpio8>;
type EncoderY = encoder::Encoder<gpio::bank0::Gpio13, gpio::bank0::Gpio14>;

/// Since we're always accessing these pins together we'll store them in a tuple.
/// Giving this tuple a type alias means we won't need to use () when putting them
/// inside an Option. That will be easier to read.
type InterruptObjs = (LedPin, EncoderX, EncoderY);

/// This how we transfer our pins to the Interrupt Handler.
/// We'll have the option hold both using the InterruptObjs type.
/// This will make it a bit easier to unpack them later.
static GLOBAL_PINS: Mutex<RefCell<Option<InterruptObjs>>> = Mutex::new(RefCell::new(None));
static GLOBAL_VALUES: Mutex<RefCell<Option<(i16, i16)>>> = Mutex::new(RefCell::new(None));

// For gpio toggle()
use embedded_hal::digital::StatefulOutputPin;

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
    let led_a: LedPin = pins.gpio11.reconfigure();
    let pin_x1 = pins.gpio7.into_pull_up_input();
    let pin_x2 = pins.gpio8.into_pull_up_input();
    let pin_y1 = pins.gpio13.into_pull_up_input();
    let pin_y2 = pins.gpio14.into_pull_up_input();

    let mut encoder_x = encoder::Encoder::new(pin_x1, pin_x2);
    let mut encoder_y = encoder::Encoder::new(pin_y1, pin_y2);
    encoder_x.enable_interrupts();
    encoder_y.enable_interrupts();

    // Set up the MPU 9250 IMU over I2C
    let i2c_sda = pins.gpio4.into_function::<hal::gpio::FunctionI2C>();
    let i2c_scl = pins.gpio5.into_function::<hal::gpio::FunctionI2C>();
    // TODO: move baudrates to constants
    let i2c = hal::i2c::I2C::new_controller(
        pac.I2C0,
        i2c_sda,
        i2c_scl,
        100.kHz(),
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
    );
    let mut imu = mpu_9250::MPU9250::new(i2c, mpu_9250::MPU9250_ADDRESS_AD0);

    // Verify we're talking to the MPU 9250
    let x: u8 = imu.read_byte(mpu_9250::MPU9250_ADDRESS_AD0, mpu_9250::WHO_AM_I_MPU9250);
    if x != 0x71 {
        loop {
            // infinite error blink
            led_pin.set_low().unwrap();
            delay.delay_ms(100);
            led_pin.set_high().unwrap();
            delay.delay_ms(100);
        }
    }

    // Set up RA8875 LCD driver over SPI
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

    // Verify we're talking to the RA8875
    let x: u8 = tft.read_reg(0);
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
    // TODO: verify delay. Adafruit library does 500ms but 50 seems to work
    delay.delay_ms(50);
    // done with begin() function

    tft.display_on(true);
    // Enable TFT - display enable tied to GPIOX
    tft.gpio_x(true);
    // PWM output for backlight
    tft.pwm1_config(true, ra8875::RA8875_PWM_CLK_DIV1024);
    tft.pwm1_out(255);

    // debug
    tft.fill_screen(ra8875::RA8875_YELLOW);
    delay.delay_ms(1000);

    // Give away our pins by moving them into the `GLOBAL_PINS` variable.
    // We won't need to access them in the main thread again
    critical_section::with(|cs| {
        GLOBAL_PINS
            .borrow(cs)
            .replace(Some((led_a, encoder_x, encoder_y)));
        GLOBAL_VALUES.borrow(cs).replace(Some((25, 25)));
    });

    // Unmask the IO_BANK0 IRQ so that the NVIC interrupt controller
    // will jump to the interrupt function when the interrupt occurs.
    // We do this last so that the interrupt can't go off while
    // it is in the middle of being configured
    unsafe {
        pac::NVIC::unmask(pac::Interrupt::IO_IRQ_BANK0);
    }

    // Go back to doing LCD stuff
    // TODO: make this a const
    tft.fill_screen(0xAD6F);
    led_pin.set_low().unwrap();
    delay.delay_ms(1000);

    loop {
        let mut x_pos: i16 = 0;
        let mut y_pos: i16 = 0;

        critical_section::with(|cs| {
            if let Some(ref mut value) = GLOBAL_VALUES.borrow(cs).borrow_mut().deref_mut() {
                (x_pos, y_pos) = *value;
            }
        });

        tft.draw_pixel(
            x_pos % ra8875::LCD_WIDTH,
            y_pos % ra8875::LCD_HEIGHT,
            ra8875::RA8875_BLACK,
        );
        delay.delay_ms(10)
    }
}

#[interrupt]
fn IO_IRQ_BANK0() {
    // The `#[interrupt]` attribute covertly converts this to `&'static mut Option<InterruptObjs>`
    static mut INTERRUPT_PINS: Option<InterruptObjs> = None;

    // This is one-time lazy initialisation. We steal the variables given to us
    // via `GLOBAL_PINS`.
    if INTERRUPT_PINS.is_none() {
        critical_section::with(|cs| {
            *INTERRUPT_PINS = GLOBAL_PINS.borrow(cs).take();
        });
    }

    if let Some(gpios) = INTERRUPT_PINS {
        // borrow led and button by *destructuring* the tuple
        // these will be of type `&mut XXPin` so we don't have
        // to move them back into the static after we use them
        let (led_a, encoder_x, encoder_y) = gpios;

        // Process encoder 1 (X)
        let mut result_x = encoder::DIR_NONE;
        if encoder_x.pin1.interrupt_status(gpio::Interrupt::EdgeHigh)
            | encoder_x.pin1.interrupt_status(gpio::Interrupt::EdgeLow)
            | encoder_x.pin2.interrupt_status(gpio::Interrupt::EdgeHigh)
            | encoder_x.pin2.interrupt_status(gpio::Interrupt::EdgeLow)
        {
            result_x = encoder_x.process();
            encoder_x.pin1.clear_interrupt(gpio::Interrupt::EdgeHigh);
            encoder_x.pin1.clear_interrupt(gpio::Interrupt::EdgeLow);
            encoder_x.pin2.clear_interrupt(gpio::Interrupt::EdgeHigh);
            encoder_x.pin2.clear_interrupt(gpio::Interrupt::EdgeLow);
        }

        // Process encoder 2 (Y)
        let mut result_y = encoder::DIR_NONE;
        if encoder_y.pin1.interrupt_status(gpio::Interrupt::EdgeHigh)
            | encoder_y.pin1.interrupt_status(gpio::Interrupt::EdgeLow)
            | encoder_y.pin2.interrupt_status(gpio::Interrupt::EdgeHigh)
            | encoder_y.pin2.interrupt_status(gpio::Interrupt::EdgeLow)
        {
            result_y = encoder_y.process();
            encoder_y.pin1.clear_interrupt(gpio::Interrupt::EdgeHigh);
            encoder_y.pin1.clear_interrupt(gpio::Interrupt::EdgeLow);
            encoder_y.pin2.clear_interrupt(gpio::Interrupt::EdgeHigh);
            encoder_y.pin2.clear_interrupt(gpio::Interrupt::EdgeLow);
        }

        critical_section::with(|cs| {
            if let Some(ref mut xy) = GLOBAL_VALUES.borrow(cs).borrow_mut().deref_mut() {
                // Update X value
                if result_x == encoder::DIR_CW {
                    (*xy).0 += 1;
                } else if result_x == encoder::DIR_CCW {
                    (*xy).0 -= 1;
                }
                // Update Y value
                if result_y == encoder::DIR_CW {
                    (*xy).1 += 1;
                } else if result_y == encoder::DIR_CCW {
                    (*xy).1 -= 1;
                }
            }
        });
    }
}

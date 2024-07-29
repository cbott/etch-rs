// Functions for processing inputs from a rotary encoder
// Testing out https://github.com/buxtronix/arduino/blob/master/libraries/Rotary/Rotary.cpp
use bsp::hal::gpio::{self, FunctionSioInput, Pin, PullUp};
use core::convert::Infallible;
use embedded_hal::digital::InputPin;
use rp_pico::{self as bsp, hal::gpio::PinId};
// Interrupt macro
use bsp::hal::pac::interrupt;
use core::ops::DerefMut;

// Values returned by 'process'
// No complete step yet.
pub const DIR_NONE: u8 = 0x0;
// Clockwise step.
pub const DIR_CW: u8 = 0x10;
// Anti-clockwise step.
pub const DIR_CCW: u8 = 0x20;

// Use the half-step state table (emits a code at 00 and 11)
const R_START: u8 = 0x0;
const R_CCW_BEGIN: u8 = 0x1;
const R_CW_BEGIN: u8 = 0x2;
const R_START_M: u8 = 0x3;
const R_CW_BEGIN_M: u8 = 0x4;
const R_CCW_BEGIN_M: u8 = 0x5;

const TTABLE: [[u8; 4]; 6] = [
    // R_START (00)
    [R_START_M, R_CW_BEGIN, R_CCW_BEGIN, R_START],
    // R_CCW_BEGIN
    [R_START_M | DIR_CCW, R_START, R_CCW_BEGIN, R_START],
    // R_CW_BEGIN
    [R_START_M | DIR_CW, R_CW_BEGIN, R_START, R_START],
    // R_START_M (11)
    [R_START_M, R_CCW_BEGIN_M, R_CW_BEGIN_M, R_START],
    // R_CW_BEGIN_M
    [R_START_M, R_START_M, R_CW_BEGIN_M, R_START | DIR_CW],
    // R_CCW_BEGIN_M
    [R_START_M, R_CCW_BEGIN_M, R_START_M, R_START | DIR_CCW],
];

pub struct Encoder<I1, I2>
where
    I1: PinId,
    I2: PinId,
{
    pub pin1: Pin<I1, FunctionSioInput, gpio::PullUp>,
    pub pin2: Pin<I2, FunctionSioInput, gpio::PullUp>,
    state: u8,
}

impl<I1, I2> Encoder<I1, I2>
where
    I1: PinId,
    I2: PinId,
{
    pub fn new(
        pin1: Pin<I1, gpio::FunctionSio<gpio::SioInput>, PullUp>,
        pin2: Pin<I2, gpio::FunctionSio<gpio::SioInput>, PullUp>,
    ) -> Self {
        Self {
            pin1: pin1,
            pin2: pin2,
            state: R_START,
        }
    }

    pub fn enable_interrupts(&mut self) {
        // TODO: either remove this and handle all interrupt things in main
        // or else pull in all interrupt handling code. Weird to mix them.
        self.pin1
            .set_interrupt_enabled(gpio::Interrupt::EdgeLow, true);
        self.pin1
            .set_interrupt_enabled(gpio::Interrupt::EdgeHigh, true);
        self.pin2
            .set_interrupt_enabled(gpio::Interrupt::EdgeLow, true);
        self.pin2
            .set_interrupt_enabled(gpio::Interrupt::EdgeHigh, true);
    }

    pub fn process(&mut self) -> u8 {
        let pinstate: u8 = if self.pin1.is_high().unwrap() {
            0
        } else {
            0b01
        } | if self.pin2.is_high().unwrap() {
            0
        } else {
            0b10
        };
        // Determine new state from the pins and state table.
        self.state = TTABLE[(self.state & 0xF) as usize][pinstate as usize];
        // Return emit bits, ie the generated event.
        return self.state & 0x30;
    }
}

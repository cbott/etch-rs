// Functions for processing inputs from a rotary encoder
// Testing out https://github.com/buxtronix/arduino/blob/master/libraries/Rotary/Rotary.cpp
use bsp::hal::gpio::{self, FunctionSioInput, Pin, PullUp};
use core::convert::Infallible;
use embedded_hal::digital::InputPin;
use rp_pico as bsp;
// Interrupt macro
use bsp::hal::pac::interrupt;
use core::ops::DerefMut;

// Values returned by 'process'
// No complete step yet.
const DIR_NONE: u8 = 0x0;
// Clockwise step.
const DIR_CW: u8 = 0x10;
// Anti-clockwise step.
const DIR_CCW: u8 = 0x20;

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

pub struct Encoder<P1, P2>
where
    P1: InputPin<Error = Infallible>,
    P2: InputPin<Error = Infallible>,
{
    pin1: P1,
    pin2: P2,
    state: u8,
}

impl<P1, P2> Encoder<P1, P2>
where
    P1: InputPin<Error = Infallible>,
    P2: InputPin<Error = Infallible>,
{
    pub fn new(pin1: P1, pin2: P2) -> Self {
        Self {
            pin1: pin1,
            pin2: pin2,
            state: R_START,
        }
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

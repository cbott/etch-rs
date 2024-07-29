//! Library for the MPU 9250 IMU
//! https://www.sparkfun.com/products/retired/13762
#![no_std]
#![no_main]

use embedded_hal::i2c::I2c;

mod constants;
// re-export constants for use outside the library
pub use constants::*;

pub struct MPU9250<I2C>
where
    I2C: I2c,
{
    adr: u8, // I2C address
    i2c_controller: I2C,
}

impl<I2C> MPU9250<I2C>
where
    I2C: I2c,
{
    // Initialize a new MPU9250 with I2C communication
    pub fn new(i2c: I2C, address: u8) -> Self {
        Self {
            adr: address,
            i2c_controller: i2c,
        }
    }

    // Read a byte from the given register address from device using I2C
    pub fn read_byte(&mut self, device_adr: u8, register_adr: u8) -> u8 {
        // TODO: could change this to write_read
        // Write the register address
        self.i2c_controller
            .write(device_adr, &[register_adr])
            .unwrap();

        // Read one byte from the slave register address
        let mut readbuf: [u8; 1] = [0; 1];
        self.i2c_controller.read(self.adr, &mut readbuf).unwrap();

        // Return the data read from the slave register
        readbuf[0]
    }

    pub fn write_byte(&mut self, device_adr: u8, register_adr: u8, data: u8) -> u8 {
        // _wire->setClock(_interfaceSpeed);			// Reset to the desired speed, in case other devices required a slowdown
        // _wire->beginTransmission(deviceAddress);  	// Initialize the Tx buffer
        // _wire->write(registerAddress);      		// Put slave register address in Tx buffer
        // _wire->write(data);                 		// Put data in Tx buffer
        // _wire->endTransmission();           		// Send the Tx buffer
        self.i2c_controller
            .write(device_adr, &[register_adr, data])
            .unwrap();
        // TODO: Fix this to return something meaningful
        // maybe success bool?
        0
    }

    // Read 1 or more bytes from given register and device using I2C
    pub fn read_bytes(&mut self, device_adr: u8, register_adr: u8, count: u8, dest: &[u8]) -> u8 {
        // Put slave register address in Tx buffer
        self.i2c_controller
            .write(device_adr, &[register_adr])
            .unwrap();
        let mut i: u8 = 0;
        // Read bytes from slave register address
        // _wire->requestFrom(deviceAddress, count);
        // while (_wire->available())
        // {
        // // Put read results in the Rx buffer
        // dest[i++] = _wire->read();
        // }

        return i; // Return number of bytes written
    }
}

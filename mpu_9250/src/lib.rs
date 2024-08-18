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
    g_scale: GyroscopeScale,     // Full scale gyroscope range
    a_scale: AccelerometerScale, // Full scale accelerometer range
}

#[derive(Copy, Clone)]
enum AccelerometerScale {
    Afs2g = 0,
    Afs4g,
    Afs8g,
    Afs16g,
}

#[derive(Copy, Clone)]
enum GyroscopeScale {
    Gfs250DPS = 0,
    Gfs500DPS,
    Gfs1000DPS,
    Gfs2000DPS,
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
            g_scale: GyroscopeScale::Gfs250DPS,
            a_scale: AccelerometerScale::Afs2g,
        }
    }

    // Read a byte from the given register address from device using I2C
    pub fn read_byte(&mut self, device_adr: u8, register_adr: u8) -> u8 {
        let mut readbuf: [u8; 1] = [0; 1];
        self.i2c_controller
            .write_read(device_adr, &[register_adr], &mut readbuf)
            .unwrap();

        // Return the data read from the slave register
        readbuf[0]
    }

    // Write a single byte to the given register using I2C
    pub fn write_byte(&mut self, device_adr: u8, register_adr: u8, data: u8) -> u8 {
        self.i2c_controller
            .write(device_adr, &[register_adr, data])
            .unwrap();
        // TODO: Fix this to return something meaningful
        // maybe success bool?
        0
    }

    // Read 1 or more bytes from given register and device using I2C
    pub fn read_bytes(&mut self, device_adr: u8, register_adr: u8, dest: &mut [u8]) {
        self.i2c_controller
            .write_read(device_adr, &[register_adr], dest)
            .unwrap();
    }

    // Initialize the MPU 9250
    pub fn init(&mut self) {
        // TODO: remove leftover comments from original library
        // wake up device
        // Clear sleep mode bit (6), enable all sensors
        self.write_byte(self.adr, PWR_MGMT_1, 0x00);
        // Wait for all registers to reset
        // ~100ms (Pico clock is 125MHz)
        cortex_m::asm::delay(12_500_000);

        // Get stable time source
        // Auto select clock source to be PLL gyroscope reference if ready else
        self.write_byte(self.adr, PWR_MGMT_1, 0x01);
        cortex_m::asm::delay(25_000_000);

        // Configure Gyro and Thermometer
        // Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz,
        // respectively;
        // minimum delay time for this setting is 5.9 ms, which means sensor fusion
        // update rates cannot be higher than 1 / 0.0059 = 170 Hz
        // DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
        // With the MPU9250, it is possible to get gyro sample rates of 32 kHz (!),
        // 8 kHz, or 1 kHz
        self.write_byte(self.adr, CONFIG, 0x03);

        // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
        // Use a 200 Hz rate; a rate consistent with the filter update rate
        // determined inset in CONFIG above.
        self.write_byte(self.adr, SMPLRT_DIV, 0x04);

        // Set gyroscope full scale range
        // Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are
        // left-shifted into positions 4:3

        // get current GYRO_CONFIG register value
        let mut c: u8 = self.read_byte(self.adr, GYRO_CONFIG);
        // c = c & ~0xE0; // Clear self-test bits [7:5]
        c = c & !0x02; // Clear Fchoice bits [1:0]
        c = c & !0x18; // Clear AFS bits [4:3]
        c = c | ((self.g_scale as u8) << 3); // Set full scale range for the gyro
                                             // Set Fchoice for the gyro to 11 by writing its inverse to bits 1:0 of
                                             // GYRO_CONFIG
                                             // c =| 0x00;
                                             // Write new GYRO_CONFIG value to register
        self.write_byte(self.adr, GYRO_CONFIG, c);

        // Set accelerometer full-scale range configuration
        // Get current ACCEL_CONFIG register value
        c = self.read_byte(self.adr, ACCEL_CONFIG);
        // c = c & ~0xE0; // Clear self-test bits [7:5]
        c = c & !0x18; // Clear AFS bits [4:3]
        c = c | ((self.a_scale as u8) << 3); // Set full scale range for the accelerometer
                                             // Write new ACCEL_CONFIG register value
        self.write_byte(self.adr, ACCEL_CONFIG, c);

        // Set accelerometer sample rate configuration
        // It is possible to get a 4 kHz sample rate from the accelerometer by
        // choosing 1 for accel_fchoice_b bit [3]; in this case the bandwidth is
        // 1.13 kHz
        // Get current ACCEL_CONFIG2 register value
        c = self.read_byte(self.adr, ACCEL_CONFIG2);
        c = c & !0x0F; // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
        c = c | 0x03; // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
                      // Write new ACCEL_CONFIG2 register value
        self.write_byte(self.adr, ACCEL_CONFIG2, c);
        // The accelerometer, gyro, and thermometer are set to 1 kHz sample rates,
        // but all these rates are further reduced by a factor of 5 to 200 Hz because
        // of the SMPLRT_DIV setting

        // Configure Interrupts and Bypass Enable
        // Set interrupt pin active high, push-pull, hold interrupt pin level HIGH
        // until interrupt cleared, clear on read of INT_STATUS, and enable
        // I2C_BYPASS_EN so additional chips can join the I2C bus and all can be
        // controlled by the Arduino as master.
        self.write_byte(self.adr, INT_PIN_CFG, 0x22);
        // Enable data ready (bit 0) interrupt
        self.write_byte(self.adr, INT_ENABLE, 0x01);
    }

    // If intPin goes high, all data registers have new data
    pub fn has_data(&mut self) -> bool {
        self.read_byte(self.adr, INT_STATUS) & 0x01 != 0
    }

    // get resolution of accelerometer in g's per bit, based on configured full scale range
    fn get_accelerometer_resolution(&self) -> f32 {
        // TODO: could make this a property of MPU9250 and pre-calculate it
        // ADCs are 16 bit, divide by 2 for full scale range (1<<16 / 2 = 32768)
        (match self.a_scale {
            AccelerometerScale::Afs2g => 2.0,
            AccelerometerScale::Afs4g => 4.0,
            AccelerometerScale::Afs8g => 8.0,
            AccelerometerScale::Afs16g => 16.0,
        }) / 32768.0
    }

    // read acceleration in g's, returns (x, y, z) axes
    pub fn read_accel_data(&mut self) -> (f32, f32, f32) {
        let mut rawdata: [u8; 6] = [0; 6];
        // Read the six raw data registers into data array
        self.read_bytes(self.adr, ACCEL_XOUT_H, &mut rawdata);
        let resolution = self.get_accelerometer_resolution();

        // Turn the MSB and LSB into a signed 16-bit value
        let x_counts = ((rawdata[0] as i16) << 8) | (rawdata[1] as i16);
        let x_g = (x_counts as f32) * resolution;

        let y_counts = ((rawdata[2] as i16) << 8) | (rawdata[3] as i16);
        let y_g = (y_counts as f32) * resolution;

        let z_counts = ((rawdata[4] as i16) << 8) | (rawdata[5] as i16);
        let z_g = (z_counts as f32) * resolution;

        (x_g, y_g, z_g)
    }
}

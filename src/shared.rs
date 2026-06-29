use embassy_stm32::i2c::I2c;
use embassy_stm32::peripherals::{I2C1, I2C2, I2C3};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::channel::Channel;
use static_cell::StaticCell;
use embassy_sync::mutex::Mutex;
use embassy_stm32::mode::Async;
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_stm32::spi::Spi;
use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;
use embassy_stm32::gpio::Output;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;



pub type SpiBusType = Mutex<ThreadModeRawMutex, Spi<'static, Async, embassy_stm32::spi::mode::Master>>; 
pub static SPI_BUS : StaticCell<SpiBusType> = StaticCell::new();
pub type SpiDeviceType = SpiDevice<'static, 
ThreadModeRawMutex,
Spi<'static, Async, embassy_stm32::spi::mode::Master>,
Output<'static>>;



pub type I2cBusType = Mutex<ThreadModeRawMutex, I2c<'static, Async, embassy_stm32::i2c::Master>>;
pub static I2C_BUS : StaticCell<I2cBusType> = StaticCell::new();

pub type I2cDeviceType = I2cDevice<'static, ThreadModeRawMutex, I2c<'static, Async, embassy_stm32::i2c::Master>>;


pub type I2c2BusType = Mutex<ThreadModeRawMutex, I2c<'static, Async , embassy_stm32::i2c::Master>>;
pub static I2C2_BUS : StaticCell<I2c2BusType> = StaticCell::new();
pub type I2c2DeviceType = I2cDevice<'static, ThreadModeRawMutex, I2c<'static, Async , embassy_stm32::i2c::Master>>;

pub type I2c3BusType = Mutex<ThreadModeRawMutex, I2c<'static, Async , embassy_stm32::i2c::Master>>;
pub static I2C3_BUS : StaticCell<I2c3BusType> = StaticCell::new();
pub type I2c3DeviceType = I2cDevice<'static, ThreadModeRawMutex, I2c<'static, Async , embassy_stm32::i2c::Master>>;

// ThreadModeRawMutex
// CriticalSectionRawMutex
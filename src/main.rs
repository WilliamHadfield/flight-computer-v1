#![no_std]
#![no_main]

 mod I2C;
mod logic;
mod SPI;
mod UART;
mod WATCHDOG;

use embassy_stm32::time::khz;
 use crate::I2C::gps::gps_task;
use crate::SPI::lis3mdl::{MAG_CHANNEL_LOG, magno_task};
 use crate::WATCHDOG::watchdog::{watchdog_task};
 use crate::I2C::bmp390::{BARO_CHANNEL_LOG, baro_task};
 use crate::I2C::mpu6050::{MPU_CHANNEL_LOG, mpu_task};
use crate::logic::sensor_fusion_ESKF::fusion_task;
use crate::logic::sensor_fusion_ESKF::ESKF_CHANNEL;

use embassy_stm32::{i2c::I2c, wdg::IndependentWatchdog};
use embassy_stm32::time::Hertz;
use embassy_executor::Spawner;
use embassy_stm32::{bind_interrupts, dma, i2c, peripherals, spi};
use flightcomputer::shared::{I2C_BUS, SPI_BUS};
use {defmt_rtt as _, panic_probe as _};
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_time::Timer;
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_sync::mutex::Mutex;
use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;
use embassy_stm32::spi::MODE_3;
use embassy_stm32::rcc::Sysclk;
use flightcomputer::shared::I2C2_BUS;
use flightcomputer::shared::I2C3_BUS;
bind_interrupts!(struct Irqs {
    I2C1_EV => i2c::EventInterruptHandler<peripherals::I2C1>;
    I2C1_ER => i2c::ErrorInterruptHandler<peripherals::I2C1>;
    DMA1_STREAM6 => dma::InterruptHandler<peripherals::DMA1_CH6>;
    DMA1_STREAM0 => dma::InterruptHandler<peripherals::DMA1_CH0>;
    DMA2_STREAM3 => dma::InterruptHandler<peripherals::DMA2_CH3>;
    DMA2_STREAM0 => dma::InterruptHandler<peripherals::DMA2_CH0>;
     I2C2_EV => i2c::EventInterruptHandler<peripherals::I2C2>;
    I2C2_ER => i2c::ErrorInterruptHandler<peripherals::I2C2>;
DMA1_STREAM7 => dma::InterruptHandler<peripherals::DMA1_CH7>;
DMA1_STREAM3 => dma::InterruptHandler<peripherals::DMA1_CH3>;
 I2C3_EV => i2c::EventInterruptHandler<peripherals::I2C3>;
    I2C3_ER => i2c::ErrorInterruptHandler<peripherals::I2C3>;
DMA1_STREAM4 => dma::InterruptHandler<peripherals::DMA1_CH4>;
DMA1_STREAM2 => dma::InterruptHandler<peripherals::DMA1_CH2>;
});

#[embassy_executor::main]
async fn main(spawner : Spawner) {
   let mut config = embassy_stm32::Config::default();
   
    let p = embassy_stm32::init(config);
   
   let mut i2c_config = embassy_stm32::i2c::Config::default();
   i2c_config.frequency = Hertz::hz(400_000);
   i2c_config.timeout = embassy_time::Duration::from_millis(10);
    let mut i2c = I2c::new(
        p.I2C1,
        p.PB8,
        p.PB9,
        p.DMA1_CH6,
        p.DMA1_CH0,
        Irqs,
       i2c_config,
    );

    let mut i2c2_config = embassy_stm32::i2c::Config::default();
   i2c2_config.frequency = Hertz::hz(400_000);
   i2c2_config.timeout = embassy_time::Duration::from_millis(10);
    let i2c2 = I2c::new(
p.I2C2,
p.PB10,
p.PB3,
p.DMA1_CH7,
p.DMA1_CH3,
Irqs,
i2c2_config,
    );

let mut i2c3_config = embassy_stm32::i2c::Config::default();
   i2c3_config.frequency = Hertz::hz(400_000);
   i2c3_config.timeout = embassy_time::Duration::from_millis(10);
    let i2c3 = I2c::new(
p.I2C3,
p.PA8,
p.PC9,
p.DMA1_CH4,
p.DMA1_CH2,
Irqs,
i2c3_config,
    );

    

    let mut spi_config = embassy_stm32::spi::Config::default();
     spi_config.frequency = Hertz::mhz(1);
spi_config.mode = MODE_3;
    let spi = embassy_stm32::spi::Spi::new(
      p.SPI1,
      p.PA5,
      p.PA7,
      p.PA6,
        p.DMA2_CH3,
        p.DMA2_CH0,
        Irqs,
        spi_config,
    );

    let spi_bus = SPI_BUS.init(Mutex::new(spi));
    let cs = Output::new(p.PA4, Level::High, Speed::High);
   
   let spi_device_magno = SpiDevice::new(spi_bus, cs);
  let i2c_bus = I2C_BUS.init(Mutex::new(i2c));
let i2c_device_mpu = I2cDevice::new(i2c_bus);
 
let i2c2_bus = I2C2_BUS.init(Mutex::new(i2c2));
let i2c2_device_baro = I2cDevice::new(i2c2_bus);

let i2c3_bus = I2C3_BUS.init(Mutex::new(i2c3));
let i2c3_device_gps = I2cDevice::new(i2c3_bus);


// let mut wdg = IndependentWatchdog::new(p.IWDG, 1_000_000);
//wdg.unleash();
defmt::info!("watchdog UNLEASHED");
spawner.spawn(mpu_task(i2c_device_mpu).unwrap());
Timer::after_millis(30).await;
spawner.spawn(magno_task(spi_device_magno).unwrap());
Timer::after_millis(30).await;
spawner.spawn(baro_task(i2c2_device_baro).unwrap());
Timer::after_millis(30).await;
//spawner.spawn(watchdog_task(wdg).unwrap());
Timer::after_millis(30).await;
spawner.spawn(ReadAllData().unwrap());
Timer::after_millis(30).await;
spawner.spawn(fusion_task().unwrap());
Timer::after_millis(30).await;
spawner.spawn(gps_task(i2c3_device_gps).unwrap());

}

#[embassy_executor::task]
async fn ReadAllData(){
loop {
  let data_4 = ESKF_CHANNEL.receive().await;
     defmt::info!("ESKF data : {}", data_4);
Timer::after_millis(80).await;
}
}







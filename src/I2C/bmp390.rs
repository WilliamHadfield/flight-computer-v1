#![allow(dead_code)]

use bmp390_rs::register::status;
use crate::WATCHDOG::watchdog::TASK_BARO;
use flightcomputer::shared::I2c2DeviceType;
use embassy_time::{Delay,Duration, Instant, Timer};
use embassy_sync::channel::Channel;
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use defmt::println;
use crate::WATCHDOG::watchdog::checkin;
use bmp390_rs::{Bmp390, SdoPinState, ResetPolicy};
use bmp390_rs::config::Configuration;
use bmp390_rs::register::pwr_ctrl::PowerMode;
use bmp390_rs::register::osr::Oversampling;
use bmp390_rs::register::odr::OutputDataRate;


#[derive(defmt::Format)]
#[derive(Copy, Clone)]
pub struct BaroData {
    pub pressure : f32,
    pub temperature : f32,

}
pub static BARO_CHANNEL_EKF : Channel<ThreadModeRawMutex, BaroData, 8> = Channel::new();
pub static BARO_CHANNEL_LOG : Channel<ThreadModeRawMutex, BaroData, 8> = Channel::new();
#[embassy_executor::task]
 pub async fn baro_task(i2c_bus : I2c2DeviceType) {


let mut delay = Delay;

let config = Configuration::default()
.iir_filter_coefficient(bmp390_rs::register::config::IIRFilterCoefficient::Coef0)
.pressure_oversampling(Oversampling::X2)
.temperature_oversampling(Oversampling::X1)
.output_data_rate(OutputDataRate::R25Hz);


let mut baro = Bmp390::new_i2c (
i2c_bus,
SdoPinState::High,
config,
ResetPolicy::None,
&mut delay,
).await.unwrap();

Timer::after_millis(30).await;

println!("BMP390 INitilaized");

let mode = baro.mode().await.unwrap();
match mode {
    PowerMode::Normal => defmt::info!("baro mode: normal"),
 PowerMode::Sleep => defmt::info!("baro mode: sleep"),
  PowerMode::Forced => defmt::info!("baro mode: forced"),
}
let err = baro.error_flags().await.unwrap();
defmt::info!("baro errors - fatal : {}, cmd : {}, conf: {}", err.fatal_err, err.cmd_err, err.conf_err);

Timer::after_millis(30).await;


const MAX_FAILURESBARO: u8 = 5;
let mut baro_failures = 0;
// 20hz currently for 40ms
loop {


match baro.status().await {
    Ok(status) if status.drdy_press => {
match baro.read_sensor_data().await {
    Ok(data) => {
baro_failures = 0;
if data.pressure() < 29999.0 || data.pressure() > 125000.0 {
    defmt::warn!("baro pressure out of bounds");
}
if data.temperature() < -41.0 || data.temperature() > 86.0 {
    defmt::warn!("baro temperature out of bounds");
}

  let _ = BARO_CHANNEL_LOG.sender().try_send(BaroData {
    pressure: data.pressure(),
    temperature : data.temperature(),
});
  let _ = BARO_CHANNEL_EKF.sender().try_send(BaroData {
    pressure: data.pressure(),
    temperature : data.temperature(),
});

  Timer::after_millis(40).await;

    } 
    Err(e) => {
        baro_failures += 1;
        defmt::warn!("baro read failed!");
        if baro_failures == MAX_FAILURESBARO {
            panic!("ERROR-5 CONSECUTIVE ERRORS, SENSOR FAULTY");
        }
    }
}
}
Ok(_) => {
    defmt::info!("baro not ready!");
Timer::after_millis(40).await;
}
Err(_) => {
    defmt::warn!("baro status read failed");
    Timer::after_millis(40).await;

}
}





}
}
 
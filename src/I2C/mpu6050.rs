#![allow(dead_code)]
use flightcomputer::shared::I2cDeviceType;
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{Duration,Instant,Timer};
use defmt::println;
use embedded_hal_async::i2c::I2c;
 use crate::WATCHDOG::watchdog::TASK_MPU;
use crate::WATCHDOG::watchdog::checkin;

pub struct RawImuData {
    pub rawaccel_x : i16,
    pub rawaccel_y : i16,
    pub rawaccel_z : i16,
    pub rawgyro_x : i16,
    pub rawgyro_y : i16,
    pub rawgyro_z : i16,
}



#[derive(defmt::Format)]
#[derive(Copy, Clone)]
pub struct ImuData {
    pub accel_x : f32,
    pub accel_y : f32,
    pub accel_z : f32,
    pub gyro_x : f32,
    pub gyro_y : f32,
    pub gyro_z : f32,
}


pub static MPU_CHANNEL_EKF : Channel<ThreadModeRawMutex, ImuData, 8> = Channel::new();
pub static MPU_CHANNEL_LOG : Channel<ThreadModeRawMutex, ImuData, 8> = Channel::new();




#[embassy_executor::task]
pub async fn mpu_task(mut i2c_bus : I2cDeviceType) {




    let mut buf = [0u8; 1];
    i2c_bus.write(0x68, &[0x75]).await.unwrap();
Timer::after_micros(10).await;
i2c_bus.read(0x68, &mut buf).await.unwrap();

    if buf[0] != 0x68 {
        panic!("mpu-6050 sensor not found!");
    }
    println!("mpu-6050 sensor FOUND, proceed");

    Timer::after_millis(30).await;

// wakey wakey
    i2c_bus.write(0x68, &[0x6B, 0x01]).await.unwrap();
defmt::info!("MPU6050 AWAKE");
Timer::after_millis(30).await;

// configs range to 2gs +-
i2c_bus.write(0x68, &[0x1C, 0x00]).await.unwrap();
defmt::info!("configured gs");

Timer::after_millis(30).await;

// config gyro range +- 250 degrees per sec
i2c_bus.write(0x68, &[0x1B, 0x00]).await.unwrap();
defmt::info!("configured gyro!");

Timer::after_millis(30).await;



let mut imu_failures : u8 = 0;
const MAX_FAILURES: u8 = 5;
// 50hz for 20ms
loop {

//reads 14 bytes now
let mut data = [0u8; 14];
match i2c_bus.write(0x68, &[0x3B]).await {
    Ok(_) => {},
    Err(e) => {
        defmt::warn!("IMU WRITE FAILED : {}", e);
        Timer::after_millis(10).await;
        continue;
    }
}

match i2c_bus.read(0x68, &mut data).await {
    Ok(_) => {
imu_failures = 0;



// acceleration 
let rawaccel_x = (data[0] as i16) << 8 | data[1] as i16;
let rawaccel_y = (data[2] as i16) << 8 | data[3] as i16;
let rawaccel_z = (data[4] as i16) << 8 | data[5] as i16;

// gyro
let rawgyro_x = (data[8] as i16) << 8 | data[9] as i16;
let rawgyro_y = (data[10] as i16) << 8 | data[11] as i16;
let rawgyro_z = (data[12] as i16) << 8 | data[13] as i16;

let accel_x = (rawaccel_x as f32 / 16384.0) * 9.81;
let accel_y = (rawaccel_y as f32 / 16384.0) * 9.81;
let accel_z = (rawaccel_z as f32 / 16384.0) * 9.81;

let gyro_x = (rawgyro_x as f32 / 131.0) * (core::f32::consts::PI / 180.0);
let gyro_y = (rawgyro_y as f32 / 131.0) * (core::f32::consts::PI / 180.0);
let gyro_z = (rawgyro_z as f32 / 131.0) * (core::f32::consts::PI / 180.0);

if accel_x.abs() > 156.0 || accel_y.abs() > 156.0 || accel_z.abs() > 156.0 {
    defmt::warn!("accel OUT OF RANGE {} {} {}", accel_x, accel_y, accel_z);
}

if gyro_x.abs() > 34.9 || gyro_y.abs() > 34.9 || gyro_z.abs() > 34.9 {
    defmt::warn!("gyro OUT OF RANGE {} {} {}", gyro_x, gyro_y, gyro_z);
}



let _ = MPU_CHANNEL_EKF.sender().try_send(ImuData {
    accel_x, accel_y, accel_z,
    gyro_x,gyro_y,gyro_z
});
 //checkin(TASK_MPU);
    }
 Err(e) => {
    imu_failures += 1;
    defmt::warn!("IMU read failed: {:?}", e);
    if imu_failures == MAX_FAILURES {
    panic!("5 CONSECTUTIVE ERRORS");
}
 }
}

  Timer::after_millis(20).await;

}
}




//MPU_CHANNEL_LOG.sender().send(ImuData {
//    accel_x, accel_y, accel_z,
//    gyro_x,gyro_y,gyro_z
// }).await;
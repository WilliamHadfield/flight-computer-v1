#![allow(dead_code)]

use flightcomputer::shared::SpiDeviceType;
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::Timer;
use embedded_hal_async::spi::SpiDevice;



pub struct RawMagnoData {
pub x_rgauss : i16,
pub y_rgauss : i16,
pub z_rgauss : i16,
}
#[derive(defmt::Format)]
#[derive(Copy, Clone)]
pub struct MagnoData {
pub x_gauss : f32,
pub y_gauss : f32,
pub z_gauss : f32,
}

pub static MAG_CHANNEL_EKF : Channel<ThreadModeRawMutex, MagnoData, 8> = Channel::new();
pub static MAG_CHANNEL_LOG : Channel<ThreadModeRawMutex, MagnoData, 8> = Channel::new();
#[embassy_executor::task]
pub async fn magno_task(mut spi_bus : SpiDeviceType){



    let mut buf = [0x8Fu8, 0x00u8];
    match spi_bus.transfer_in_place(&mut buf).await {
        Ok(_) => {}
        Err(e) => {
            defmt::error!("LiS3MDL not found {}", e);
            return;
        }
    }

    Timer::after_millis(30).await;
 
  defmt::info!("WHO AM I reponse : {} {}", buf[0], buf[1]);

    // very high performance XY, 80Hz ODR
    spi_bus.transfer_in_place(&mut [0x20u8, 0x7Cu8]).await.ok();

    defmt::info!("done! reg1 ");
Timer::after_millis(30).await;

    // CTRL_REG2 - +- 4 gauss full scale
   spi_bus.transfer_in_place(&mut [0x21u8, 0x00u8]).await.ok();

   
    defmt::info!("done! reg2 ");
  Timer::after_millis(30).await;

    // CTRl_REG3 - continuous measurment mode
spi_bus.transfer_in_place(&mut [0x22u8, 0x00u8]).await.ok();

    defmt::info!("done! reg3 ");
    Timer::after_millis(30).await;

    // CTRL_REG4 very high perforamnce Z axis
spi_bus.transfer_in_place(&mut [0x23u8, 0x0Cu8]).await.ok();


    defmt::info!("done! reg4 ");
 Timer::after_millis(30).await;

 
let mut magno_failures = 0;
const MAGNO_MAX: u8 = 5; 
loop {


// Burst read 6 bytes from OUT_X_L - msb set for auto increment
let mut data : [u8; 7] = [0xE8, 0, 0, 0, 0, 0, 0];
match spi_bus.transfer_in_place(&mut data).await {
    Ok(_) => {

//byte reassembly
let x_rgauss = (data[2] as i16) << 8 | data[1] as i16;
let y_rgauss = (data[4] as i16) << 8 | data[3] as i16;
let z_rgauss = (data[6] as i16) << 8 | data[5] as i16;

 // scale to gauss

 let x_gauss = x_rgauss as f32 / 6842.0;
 let y_gauss = y_rgauss as f32 / 6842.0;
 let z_gauss = z_rgauss as f32 / 6842.0;


 if x_gauss.abs() > 15.0 || y_gauss.abs() > 15.0 || z_gauss.abs() > 15.0 {
    defmt::warn!("magnometer out of RANGE! {}, {}, {}", x_gauss, y_gauss, z_gauss);
 }
 

 let _ = MAG_CHANNEL_LOG.sender().try_send(MagnoData {
    x_gauss, y_gauss, z_gauss
 });

  let _ = MAG_CHANNEL_EKF.sender().try_send(MagnoData {
    x_gauss, y_gauss, z_gauss
 });
 
}
Err(e) => {
 magno_failures += 1;
    defmt::warn!("IMU read failed: {:?}", e);
    if magno_failures == MAGNO_MAX {
  // panic!("5 CONSECTUTIVE ERRORS");
   defmt::warn!("excedded 5 consecutive errors.");
}
}
}


  Timer::after_millis(40).await;


}
}






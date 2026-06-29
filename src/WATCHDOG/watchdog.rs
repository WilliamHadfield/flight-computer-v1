use core::sync::atomic::{AtomicU32, Ordering};
use embassy_stm32::wdg::IndependentWatchdog;
use embassy_time::Timer;
use embassy_stm32::peripherals::IWDG;
use embassy_executor::Spawner;



// task ID bitmask each bit corresponds to a task, 1 = task responded, 0 = task didnt, return error watchdog get to work.
pub const TASK_MAGNO:   u32   = 1 << 0; // 01 in binary or 1
pub const TASK_BARO: u32 = 1 << 1; // 10 in binary or 2
pub const TASK_MPU: u32 = 1 << 2; // 100 in binary or 4
pub const TASK_EKF: u32 = 1 << 3; // 1000 in binary or 8
pub const TASK_SENSORS: u32 = 1 << 4; // 10000 in binary or 16


pub const ALL_TASKS: u32 = TASK_BARO
| TASK_MPU
| TASK_MAGNO
| TASK_EKF;
 // should return 11111 in binary otherwise something has gone wrong essentially just checks all task return a bit of 1. (this is the placeholder value to check against btw eg its the value that is always 11111, that the checking function will check against.)

 static TASK_CHECKINS: AtomicU32 = AtomicU32::new(0);

 pub fn checkin(task_id: u32) {
    TASK_CHECKINS.fetch_or(task_id, Ordering::Relaxed);
 }

 #[embassy_executor::task]
 pub async fn watchdog_task(mut wdg: IndependentWatchdog<'static, IWDG>) {
    loop {
        Timer::after_millis(500).await;

        let checked_in = TASK_CHECKINS.swap(0, Ordering::Relaxed);

        if checked_in == ALL_TASKS {
            wdg.pet();
            defmt::info!("WATCHDOG FED");
        }
       else {
        defmt::warn!("watchdog starved - reset incoming");
       } 
    }
 }












// | TASK_MAGNO

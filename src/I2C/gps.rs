use flightcomputer::shared::I2c3DeviceType;
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{Duration,Instant,Timer};
use defmt::println;
use embedded_hal_async::i2c::I2c;
use libm::sqrt;
use libm::cos;
use libm::sin;
use core::f64::consts::PI;


const GPS_ADDR: u8 = 0x10;
const GPS_CHUNK: usize = 32;
const A: f64 = 6_378_137.0; // sem-major axis (m)
const E2: f64 = 6.694_379_990_14e-3; // first eccenctricty squared


pub static GPS_VEL_CHANNEL_ESKF: Channel<ThreadModeRawMutex, (f32,f32), 8> = Channel::new();
pub static GPS_CHANNEL_ESKF : Channel<ThreadModeRawMutex, (f32,f32), 8> = Channel::new();


#[derive(defmt::Format, Copy, Clone)]
pub struct GpsData {
pub lat: f64,
pub lon: f64,
pub alt: f32,
pub fix: u8,
}

#[derive(defmt::Format, Copy, Clone)]
pub struct GpsVel {
    pub speed_mps : f32, // ground speed m/s
    pub course_deg: f32, // direction of travel, degrees
}

fn deg_to_rad(deg: f64) -> f64 {
deg * PI / 180.0
}

 // essentially the first gps conversion from world (geodetic) to an necessary intermediate measurment (ecef) global cartesian.

fn geodetic_to_ecef(lat_deg: f64, lon_deg : f64) -> (f64, f64, f64) {
 let phi = deg_to_rad(lat_deg);
 let lam = deg_to_rad(lon_deg);
 let sin_phi = libm::sin(phi);
 let n = A / libm::sqrt(1.0 - E2 * sin_phi * sin_phi);

 let x = n * libm::cos(phi) * libm::cos(lam);
 let y = n * libm::cos(phi) * libm::sin(lam);
 let z = n * (1.0 - E2) * sin_phi;
 (x,y,z)
}

fn parse_gga(s: &[u8]) -> Option<GpsData> {
    let mut f = [&b""[..]; 12];
    let mut n = 0;
    let mut start = 0;
    for i in 0..s.len() {
        if s[i] == b',' || s[i] == b'*' {
            if n < f.len() { f[n] = &s[start..i]; n+= 1; }
            start = i + 1;
        }
    }

    let fix = atoi(f[6]) as u8;
    let lat = nmea_deg(f[2], f[3], 2)?;
    let lon = nmea_deg(f[4], f[5], 3)?;
    let alt = atoi(f[9]) as f32;

    Some(GpsData { lat, lon, alt, fix})
}

fn geodetic_to_enu(
    lat_deg : f64,
    lon_deg : f64,
    ref_lat_deg : f64,
    ref_lon_deg : f64,
    ref_ecef : (f64,f64,f64)
) -> (f32, f32) {
let (x,y,z) = geodetic_to_ecef(lat_deg, lon_deg);
let (x0,y0,z0) = ref_ecef;

let dx = x -x0;
let dy = y - y0;
let dz = z - z0;

let phi0 = deg_to_rad(ref_lat_deg);
let lam0 = deg_to_rad(ref_lon_deg);

let sin_phi0 = libm::sin(phi0);
let cos_phi0 = libm::cos(phi0);
let sin_lam0 = libm::sin(lam0);
let cos_lam0 = libm::cos(lam0);

let east = -sin_lam0 * dx + cos_lam0 * dy;
let north = -sin_phi0 * cos_lam0 * dx - sin_phi0 * sin_lam0 * dy + cos_phi0 * dz;

(east as f32, north as f32)

}

fn nmea_deg(field : &[u8], hemi: &[u8], deg_digits: usize) -> Option<f64> {
    if field.len() < deg_digits { return None; }
    let raw = atof(field);
    let deg = (raw / 100.0) as i64 as f64; // trunucation, here were just getting rid of the mintue part of the degree-mintue almgmation that gps's give you.
    let mut dd = deg + (raw - deg * 100.0) / 60.0;
    if hemi.first() == Some(&b'S') || hemi.first() == Some(&b'W') {dd = -dd; }
    Some(dd)

}

fn atof(b: &[u8]) -> f64 { // ASCII -> float
    core::str::from_utf8(b).ok().and_then(|s| s.parse().ok()).unwrap_or(0.0)
}

fn atoi(b: &[u8]) -> i64 { // ASCII to integer
      core::str::from_utf8(b).ok().and_then(|s| s.parse().ok()).unwrap_or(0)
}

fn checksum_ok(s: &[u8]) -> bool {
    let star = match s.iter().position(|&b| b == b'*') {
        Some(i) => i,
        None => return false,
    };
    if star + 2 >= s.len() {return false;}
    let mut cs = 0u8;
    for &b in &s[1..star] {
        cs ^= b; // ^= b is equivalent to cs ^ b so cs = cs ^ b
    }
    let sent = u8::from_str_radix(
        core::str::from_utf8(&s[star+1..star+3]).unwrap_or(""), 16
    ).unwrap_or(0xFF);
    cs == sent
    
}

//vtg equivalent parser
fn parse_vtg(s: &[u8]) -> Option<GpsVel> {
    let mut f = [&b""[..]; 12];
    let mut n = 0;
    let mut start = 0;
    for i in 0..s.len() {
        if s[i] == b',' || s[i] == b'*' {
            if n < f.len() { f[n] = &s[start..i]; n += 1;}
            start = i + 1;
        }
    }

    let course = atof(f[1]) as f32; // course in degrees
    let kmh = atof(f[7]) as f32;
    let speed_mps = kmh / 3.6; // convers it to m/s

    Some(GpsVel { speed_mps, course_deg : course})
}


#[embassy_executor::task]
pub async fn gps_task(mut i2c_bus : I2c3DeviceType) {
    let mut line = [0u8; 100];
    let mut idx = 0usize;
    let mut last_char = 0u8;
    
    let ref_lat = 51.563968_f64;
    let ref_lon = -0.6128201_f64;

    let rec_ecef = geodetic_to_ecef(ref_lat, ref_lon);

    // one time config
    // 5hz fix rate
    let set_5hz = b"$PMTK220,200*2C\r\n";
    i2c_bus.write(GPS_ADDR, set_5hz).await.ok();
    Timer::after_millis(30).await;

    // enable GGA for velocity + VTG for position
    let set_output = b"$PMTK314,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n";
    i2c_bus.write(GPS_ADDR, set_output).await.ok();
    Timer::after_millis(30).await;

    loop {
    let mut chunk = [0u8; GPS_CHUNK];
    // plain read no register write, 
    i2c_bus.read(GPS_ADDR, &mut chunk).await.ok();

    for &c in &chunk {
        // filler handling drop standalone 0x0A,
        // 0x0A is just a new line.
        if c == 0x0A && last_char != 0x0D {
            last_char = c;
            continue;
        }
        last_char = c;

        if c == b'$' {
            idx = 0;
            line[idx] = c;
            idx += 1;
        } else if c == b'\n' {
            if checksum_ok(&line[..idx]) && idx > 6 && &line[3..6] == b"GGA" {
                if let Some(gps) = parse_gga(&line[..idx]) {
               let gps_lat = gps.lat;
               let gps_lon = gps.lon;
               let (east_p, north_p) = geodetic_to_enu(gps_lat, gps_lon, ref_lat, ref_lon, rec_ecef);


                let _ = GPS_CHANNEL_ESKF.sender().try_send((east_p, north_p));

                }
            } else if idx > 6 && &line[3..6] == b"VTG" {
                if let Some(vel) = parse_vtg(&line[..idx]) {
                    let course_rad = vel.course_deg as f64 * PI / 180.0;
                   
                    let v_east = vel.speed_mps as f64 * libm::sin(course_rad);
                    let v_north = vel.speed_mps as f64 * libm::cos(course_rad);
                   

                    let _ = GPS_VEL_CHANNEL_ESKF.sender().try_send((v_east as f32, v_north as f32));
                }
            }
            idx = 0;
            
        } else if idx > 0 && idx < line.len() {
            line[idx] = c;
            idx += 1;
        }
    } 
Timer::after_millis(100).await;


    }
}
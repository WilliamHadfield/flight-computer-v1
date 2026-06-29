use nalgebra::{SMatrix, SVector};
use std::fs::File;
use csv::Reader;
use anyhow::Result;
use serde::Deserialize;
use serde::Serialize;



const N_NOM: usize = 16;
type NominalState = SVector<f32, N_NOM>;

const N_ERR: usize = 15;
type ErrorState = SVector<f32, N_ERR>;
type ErrorCov = SMatrix<f32, N_ERR, N_ERR>;


// Barometer
const M_BARO: usize = 1;
type BaroMeasVec = SVector<f32, M_BARO>;
type BaroMeasMat = SMatrix<f32, M_BARO, M_BARO>;
type BaroJacobian = SMatrix<f32, M_BARO, N_ERR>;
type BaroKalmanGain = SMatrix<f32, N_ERR, M_BARO>;


// magnometer
const M_MAG: usize = 3;
type MagMeasVec = SVector<f32, M_MAG>;
type MagMeasMat = SMatrix<f32, M_MAG, M_MAG>;
type MagJacobian = SMatrix<f32, M_MAG, N_ERR>;
type MagkalmanGain = SMatrix<f32, N_ERR, M_MAG>;

const M_IMU: usize = 6;
type ImuMeasVec = SVector<f32, M_IMU>;

// Eq 3 process noise

const N_Noise: usize = 12;
type ProcessNoise = SMatrix<f32, N_Noise, N_Noise>; // 12x12 for process noise
type NoiseJacobian = SMatrix<f32, N_ERR, N_Noise>; // 15x12 for conversion

// gps
const M_GPS: usize = 4;
type GpsMeasVec = SVector<f32, M_GPS>;
type GpsMeasMat = SMatrix<f32, M_GPS, M_GPS>;
type GpsJacobian = SMatrix<f32, M_GPS, N_ERR>;
type GpsKalmanGain = SMatrix<f32, N_ERR, M_GPS>;



#[derive(Debug)]
pub struct ESKF {
    // nominal state (16)
    pub x : NominalState,

    // error state covariance (15x15)
    pub p : ErrorCov,

    // process noise 12x12
    pub Q : ProcessNoise,
    
    pub R_baro: BaroMeasMat,

    pub R_mag: MagMeasMat,

    pub R_gps: GpsMeasMat,
}


#[derive(Copy, Clone)]
pub struct ESKFState {
postion_x : f32,
position_y : f32,
position_z : f32,
velocity_x : f32,
velocity_y: f32,
velocity_z: f32,
q_rotations_scalar : f32, // scalar must always be close to or equal to 1
q_rotations_1 : f32, // i
q_rotations_2 : f32, // j
q_rotations_3 : f32, // k
accel_bias_x : f32,
accel_bias_y : f32,
accel_bias_z : f32,
gyro_bias_x : f32,
gyro_bias_y : f32,
gyro_bias_z : f32,
}
impl ESKF {
    pub fn new() -> Self {
        Self {
            x: {
                let mut x = NominalState::zeros();
                x[6] = 1.0;
                x[13] = -0.063;
                x[14] = -0.008;
                x[15] = -0.009;
                x
            },

            p: {
                let mut p = ErrorCov::zeros();
// position best intial guess
p[(0,0)] = 2.0;  p[(1,1)] = 2.0; p[(2,2)] = 2.0;

 // velocity best intial guess
 p[(3,3)] = 2.0; p[(4,4)] = 2.0; p[(5,5)] = 2.0;

 // orientation error: best intial guess
 p[(6,6)] = 1e-3;  p[(7,7)] = 1e-3; p[(8,8)] = 1e-3;

 // accel bias best intial guess
 p[(9,9)] = 0.01; p[(10,10)] = 0.01; p[(11,11)] = 0.01;
 
 // gyro bias best intial guess
 p[(12,12)] = 7.6e-3; p[(13,13)] = 7.6e-3; p[(14,14)] = 7.6e-3;
 p
            },
        Q: {
            let mut q = ProcessNoise::zeros();
            // accel noise allan deviation
            q[(0,0)] = 6.35e-5; q[(1,1)] = 6.28e-5; q[(2,2)] = 1.12e-4;
            // gyro noise allan deviation
            q[(3,3)] = 1.76e-7; q[(4,4)] = 1.91e-7; q[(5,5)] = 1.70e-7;
            // accel bias random walk
            q[(6,6)] = 1.73e-7; q[(7,7)] = 9.50e-8; q[(8,8)] = 8.77e-7;
            // gyro bias random walk
            q[(9,9)] = 2.17e-9;  q[(10,10)] = 6.91e-10; q[(11,11)] = 2.51e-10;
            q //warning your R values for gyro and accel could slightly be off because forget to factor sample rate which was 14.6hz ish and your supposed R values were done at 1hz
        },

        R_baro: {
            let mut r = BaroMeasMat::zeros();
            r[(0,0)] = 0.053;
            r
        },

        R_mag: {
            let mut r = MagMeasMat::zeros();
            r[(0,0)] = 2.5e-4;
            r[(1,1)] = 2.5e-4;
            r[(2,2)] = 2.5e-4;
            r
        },

         R_gps: {
        let mut r = GpsMeasMat::zeros();
        r[(0,0)] = 4.0;   r[(1,1)] = 4.0; // east_p variance north_p variance
         r[(2,2)] = 0.25;  r[(3,3)] = 0.25; // east_v variance north_v variance
         r
        },
        }

       
    }

    pub fn predict(&mut self, imu: ImuMeasVec, dt: f32) {
    // extract nominal state
    let px = self.x[0]; let py = self.x[1]; let pz = self.x[2];
    let vx = self.x[3]; let vy = self.x[4]; let vz = self.x[5];
    let q0 = self.x[6];  let q1 = self.x[7]; let q2 = self.x[8]; 
    let q3 = self.x[9]; let abx = self.x[10]; let aby = self.x[11];
    let abz = self.x[12]; let gbx = self.x[13]; let gby = self.x[14];
    let gbz = self.x[15];

    // extract IMU
    let ax = imu[0]; let ay = imu[1]; let az = imu[2];
    let wx = imu[3];  let wy = imu[4];  let wz = imu[5];

    // subtract bias estimates
    let ax_c = ax - abx; let ay_c = ay - aby; let az_c = az - abz;
    let wx_c = wx-gbx; let wy_c = wy - gby; let wz_c = wz - gbz;

    // rotation matrix from body to world (R = q x v x q* sandwich quartenion method)
    let r00 = 1.0 - 2.0 * (q2 * q2 + q3 * q3);
    let r01 = 2.0 * (q1 * q2 - q0 * q3);
    let r02 = 2.0 * ( q1 * q3 + q0 * q2);

    let r10 = 2.0 * ( q1 * q2 + q0 * q3);
    let r11 = 1.0 - 2.0 * (q1*q1 + q3 * q3);
    let r12 = 2.0 * (q2 * q3 - q0 * q1);

    let r20 = 2.0 * (q1 * q3 - q0 * q2);
    let r21 = 2.0 * (q2 * q3 + q0 * q1);
    let r22 = 1.0 - 2.0 * (q1 * q1 + q2 * q2);

      // rotated corrected accel into world frame
    let ax_w = r00 * ax_c + r01 * ay_c + r02 * az_c;
    let ay_w = r10 * ax_c + r11 * ay_c + r12 * az_c;
    let az_w = r20 * ax_c + r21 * ay_c + r22 * az_c;
    
    // block 4 - position and velocity update
    // gravity in world frame
    let g: f32 = 9.81;

   
    // velocity update v+ (Rotated-a) * dt
    let new_vx = vx + (ax_w) * dt;
    let new_vy = vy + (ay_w) * dt;
    let new_vz = vz + (az_w - g) * dt;  // check if this is the correct notaition/direction for gravity eg - g is pointing up and + g is pointing downwards.
// testing with flipped sign to see what happens
    // position update = p + v * dt
    let new_px = px + new_vx * dt;
    let new_py = py + new_vy * dt;
    let new_pz = pz + new_vz * dt;

    // block 5 - quartenion update from gyro
    // corrected gyro rate as rotaton vector scaled by dt
// this turns them from radians per second to just radians
    let wx_dt = wx_c * dt;
    let wy_dt = wy_c * dt;
    let wz_dt = wz_c * dt;

    // angle of rotation or eg using pythag to get one represetnable angle from the sum in each direction.
    let angle_old = wx_dt * wx_dt + wy_dt * wy_dt + wz_dt * wz_dt;
   let angle = ((angle_old).sqrt()) as f32;

    // using small angle approximation and the e^i x pheta = cos pheta + i sin pheta to encode these rotations or aka the exponetial map.

    let (dq0, dq1, dq2, dq3) = if angle > 1e-6 {
        let s = ((angle * 0.5 ).sin()) as f32 / angle;
        (((angle * 0.5).cos()) as f32, wx_dt * s, wy_dt * s, wz_dt * s)

    } else {
        // small angle aprox
        (1.0, wx_dt * 0.5, wy_dt * 0.5, wz_dt * 0.5)
    };
    // quartnion multiply q_new = q * dq (small quartenion rotation that happened this timestep)

    let new_q0 = q0 * dq0 - q1 * dq1 - q2 * dq2 - q3 * dq3;
    let new_q1 = q0 * dq1 + q1 * dq0 + q2 * dq3 - q3 * dq2;
    let new_q2 = q0 * dq2 - q1 * dq3 + q2 * dq0 + q3 * dq1;
    let new_q3 = q0 * dq3 + q1 * dq2 - q2 * dq1 + q3 * dq0;

    // renormalise
    let norm = ((new_q0 * new_q0 + new_q1 * new_q1 + new_q2 * new_q2 + new_q3 * new_q3).sqrt()) as f32;
    let new_q0 = new_q0 / norm;
    let new_q1 = new_q1 / norm;
    let new_q2 = new_q2 / norm;
    let new_q3 = new_q3 / norm;

    // block 6 - write back to nominal state
    self.x[0] = new_px; self.x[1] = new_py; self.x[2] = new_pz;
    self.x[3] = new_vx; self.x[4] = new_vy; self.x[5] = new_vz;
    self.x[6] = new_q0; self.x[7] = new_q1; self.x[8] = new_q2;
    self.x[9] = new_q3;
    // x[10..15] biases unchanged
    
    // build fx (15x15 error state transition jacobian)
    let mut fx = ErrorCov::zeros();

    // change in position affect on change on position = identity matrix
    fx[(0,0)] = 1.0; fx[(1,1)] = 1.0; fx[(2,2)] = 1.0;
    
    // change in position afect on change in velocity is 1 x dt
    fx[(0,3)] = dt; fx[(1,4)] = dt; fx[(2,5)] = dt;

    // change in velocity affect on velcoity = I or 1.
    fx[(3,3)] = 1.0; fx[(4,4)] = 1.0; fx[(5,5)] = 1.0;

    // change in oreintation on the change in velocity = cross product of the acceleration in world frame whichs tells you how much misdirection is my acceleration getting.
   
   // skew of bod frame corrected accel? - to be confirmed 
   let s00: f32 = 0.0; let s01 = -az_c;   let s02 = ay_c;
   let s10 = az_c; let s11: f32 = 0.0;    let s12 = -ax_c;
   let s20 = -ay_c; let s21: f32 = ax_c; let s22: f32 = 0.0;
    
   // actual relationship -R x skew x dt
   fx[(3,6)] = -(r00 * s00 + r01 * s10 + r02 * s20) * dt; 
   fx[(3,7)] = -(r00 * s01 + r01 * s11 + r02 * s21) * dt; 
   fx[(3,8)] = -(r00 * s02 + r01 * s12 + r02 * s22) * dt; 

   fx[(4,6)] = -(r10 * s00 + r11 * s10 + r12 * s20) * dt; 
   fx[(4,7)] = -(r10 * s01 + r11 * s11 + r12 * s21) * dt; 
   fx[(4,8)] = -(r10 * s02 + r11 * s12 + r12 * s22) * dt; 

   fx[(5,6)] = -(r20 * s00 + r21 * s10 + r22 * s20) * dt; 
   fx[(5,7)] = -(r20 * s01 + r21 * s11 + r22 * s21) * dt; 
   fx[(5,8)] = -(r20 * s02 + r21 * s12 + r22 * s22) * dt; 

   // change in velocity impact on change in acceleration bias = - rotational matrix x dt eg need to convert velocity frame and then it has a inverse relationship with time
   fx[(3,9)] = -r00 * dt;  fx[(3,10)] = -r01 * dt;  fx[(3,11)] = -r02 * dt;
    fx[(4,9)] = -r10 * dt;  fx[(4,10)] = -r11 * dt;  fx[(4,11)] = -r12 * dt;
     fx[(5,9)] = -r20 * dt;  fx[(5,10)] = -r21 * dt;  fx[(5,11)] = -r22 * dt;

     // change in rotation affect on gyro bias inverse relationship with time
     fx[(6,12)] = -dt;
     fx[(7,13)] = -dt;
     fx[(8,14)] = -dt;

     // rotational matrix of the gyro oreination error in 3d mapping.(eg 3d error vector state)
     let dr00 = 1.0 - 2.0 * (dq2 * dq2 + dq3 * dq3);
     let dr01 = 2.0 * (dq1 * dq2 - dq0 * dq3);
     let dr02 = 2.0 * (dq1 * dq3 + dq0 * dq2);
     let dr10 = 2.0 * (dq1 * dq2 + dq0 * dq3);
     let dr11 = 1.0 - 2.0 * (dq1 * dq1 + dq3 * dq3);
     let dr12 = 2.0 * (dq2 * dq3 - dq0 * dq1);
     let dr20 = 2.0 * (dq1 * dq3 - dq0 * dq2);
     let dr21 = 2.0 * (dq2 * dq3 + dq0 * dq1);
     let dr22 = 1.0 - 2.0 * (dq1 * dq1 + dq2 * dq2);

     // change in rotations impact on change in rotations: tranpose of that matrix swap rows and columns
    fx[(6,6)] = dr00; fx[(6,7)] = dr10; fx[(6,8)] = dr20;
    fx[(7,6)] = dr01; fx[(7,7)] = dr11; fx[(7,8)] = dr21;
    fx[(8,6)] = dr02; fx[(8,7)] = dr12; fx[(8,8)] = dr22;

    // change in acceleration bias impact on acceleration bias = 1.0
    fx[(9,9)] = 1.0; fx[(10,10)] = 1.0; fx[(11,11)] = 1.0;

    // change in gyro bias impact on gyro bias = 1.0
    fx[(12,12)] = 1.0; fx[(13,13)] = 1.0; fx[(14,14)] = 1.0;


    let mut fi = NoiseJacobian::zeros();

    // matching velocity noise to velocity error it essentially converting the input velcoity noise to error.
    fi[(3,0)] = 1.0;   fi[(4,1)] = 1.0;   fi[(5,2)] = 1.0;
// matching orientation noise to oreination error measurment space to state space (error state sapce)
      fi[(6,3)] = 1.0;   fi[(7,4)] = 1.0;   fi[(8,5)] = 1.0;
     // matching acceleration bias noise to acceleration bias error (measurment space to error space or world space)
        fi[(9,6)] = 1.0;   fi[(10,7)] = 1.0;   fi[(11,8)] = 1.0;
   // gyro bias noise to gyro bias error (measurment space to error space)
          fi[(12,9)] = 1.0;   fi[(13,10)] = 1.0;   fi[(14,11)] = 1.0;


          // error covariance propgation or eg equation 3 or the final step in the predict step
          self.p = fx * self.p * fx.transpose() + fi * (self.Q * dt) * fi.transpose();

        }
        
        
        
        
        
        
        pub fn update_baro(&mut self, baro : BaroMeasVec) {
            // predicted measurment
            let h_x = self.x[2];

            // innovation/ residue calculation eg equation 1: measurment - prediction
            let y = baro[0] - h_x;

            // measurment jacobian definition
            let mut h = BaroJacobian::zeros();
            h[(0,2)] = 1.0;

            // innovation covaraince equation 2
            let s: BaroMeasMat = h * self.p * h.transpose() + self.R_baro;

            // kalman gain equation
            let s_inv = s.try_inverse().expect("S not invertible: FAILED");
            // for this .expect potentially want to do a proper match/error handling

            let k = self.p * h.transpose() * s_inv;
         
         // error state estimate calculation (the state estimate equation equivalent from the KF)
         let dx: ErrorState = k * y;
         
         // inject error into nominal state
         // positional error -> addition
         self.x[0] += dx[0];
         self.x[1] += dx[1];
         self.x[2] += dx[2];
         // velocity error -> addition you need velocity error here because position and velocity directly related as velocity the derivative.
         // or is literally mathmaticaly the rate of change of positon so therefores its error is related to positon and needs to be calculated at every baro measurment step.
         self.x[3] += dx[3];
         self.x[4] += dx[4];
         self.x[5] += dx[5];

         // oreintation error in quartenion -> 3d errors
         let dth_x = dx[6];
         let dth_y = dx[7];
         let dth_z = dx[8];
         let dth_norm = ((dth_x * dth_x + dth_y * dth_y + dth_z * dth_z)as f32).sqrt();

         let (dq0, dq1, dq2, dq3) = if dth_norm > 1e-6 {
            let s_half = ((dth_norm * 0.5).sin()) as f32 / dth_norm;
            (
              ((dth_norm * 0.5).cos()) as f32,
              dth_x * s_half,
              dth_y * s_half,
              dth_z * s_half,
            )
         } else {
         // small angle aprox
         (1.0, dth_x * 0.5, dth_y * 0.5, dth_z * 0.5)  
         };

         let q0 = self.x[6];
         let q1 = self.x[7];
         let q2 = self.x[8];
         let q3 = self.x[9];

         // q new = q (x) dq -- same multiply order as in predict (local frame rather than world frame for I cause the perbutation alligns with the states frame ) rather than global which requires a skew symetric conversion.
         let new_q0 = q0 * dq0 - q1 * dq1 - q2 * dq2 - q3 * dq3;
         let new_q1 = q0 * dq1 + q1 * dq0 + q2 * dq3 - q3 * dq2;
         let new_q2 = q0 * dq2 - q1 * dq3 + q2 * dq0 + q3 * dq1;
         let new_q3 = q0 * dq3 + q1 * dq2 - q2 * dq1 + q3 * dq0;

         // renormalise
         let norm =
        ((new_q0 * new_q0 + new_q1 * new_q1 + new_q2 * new_q2 + new_q3 * new_q3) as f32).sqrt();
         self.x[6] = new_q0 / norm;
         self.x[7] = new_q1 / norm;
         self.x[8] = new_q2 / norm;
         self.x[9] = new_q3 / norm;

         //  accel-bias error plain addition
         self.x[10] += dx[9];
         self.x[11] += dx[10];
         self.x[12] += dx[11];

         // gyro bias error -> plain addition
         self.x[13] += dx[12];
         self.x[14] += dx[13];
         self.x[15] += dx[14];

         // covariance update (joseph form like in the KF implementation)
         let i = ErrorCov::identity();
         let ikh = i - k * h;
         self.p = ikh * self.p * ikh.transpose() + k * self.R_baro * k.transpose();

        // error-state reset
        // dx has been injected into the nominal state and hence we must zero error state
        // before that though we must recalibrate P around the new nominal orientation, an analogy that resonanted with me: we have shifted the forest and now update need to update the uncertanity to fit
        // this new nominal state (the forest).
        let mut g = ErrorCov::identity();
        g[(6,7)] = 0.5 * dth_z;
        g[(6,8)] = -0.5 * dth_y;
        g[(7,6)] = -0.5 * dth_z;
        g[(7,8)] = 0.5 * dth_x;
        g[(8,6)] = 0.5 * dth_y;
        g[(8,7)] = -0.5 * dth_x;

        self.p = g * self.p * g.transpose();

        let _dx = ErrorState::zeros();


        }







        // m_world is the hardcoded known truth as comparison or the world magnetic model, which i need to feed into my system
        // note z and m_world are pre-normalized so i need to investigate if i can do that inside my from impl method and to check the world model is normalized.
        pub fn update_mag(&mut self, z: MagMeasVec) {
       
       let z_norm = ((z[0] * z[0] + z[1] * z[1] + z[2] * z[2]).sqrt()) as f32;
       let z = MagMeasVec::new(z[0] / z_norm, z[1] / z_norm, z[2] / z_norm);
       
       // hardcoded magnetic world values in gauss.
       let mut m_world = MagMeasVec::zeros();
       m_world[0] =  0.00336;
       m_world[1] =  0.19529;
       m_world[2] = -0.45065;


       let m_norm = ((m_world[0] * m_world[0] + m_world[1] * m_world[1] + m_world[2] * m_world[2]).sqrt()) as f32;
       m_world[0] = m_world[0] / m_norm;
       m_world[1] = m_world[1] / m_norm;
       m_world[2] = m_world[2] / m_norm;

        // predicted measurment 1 h(x) = R^ T * m_world
        // mag measures earth field in body frame
        // known world frame reference M_world and then rotation it into body via R^T
        let q0 = self.x[6];
        let q1 = self.x[7];
        let q2 = self.x[8];
        let q3 = self.x[9];

        // body -> world rotation matrix R
        let r00 = 1.0 - 2.0 * (q2 * q2 + q3 * q3);
        let r01 = 2.0 * (q1 * q2 - q0 * q3);
        let r02 = 2.0 * (q1 * q3 + q0 * q2);
        let r10 = 2.0 * (q1 * q2 + q0 * q3);
        let r11 = 1.0 - 2.0 * (q1 * q1 + q3 * q3);
        let r12 = 2.0 * (q2 * q3 - q0 * q1);
        let r20 = 2.0 * (q1 * q3 - q0 * q2);
        let r21 = 2.0 * (q2 * q3 + q0 * q1);
        let r22 = 1.0 - 2.0 * (q1 * q1 + q2 * q2);
        
         // h(x) = R^T * m_world R^T is world to body frame unit magnitude now.
        let mx = m_world[0];
         let my = m_world[1];
          let mz = m_world[2];

        
        let hx0 = r00 * mx + r10 * my + r20 * mz; // column 0
        let hx1 = r01 * mx + r11 * my + r21 * mz; // column 1
        let hx2 = r02 * mx + r12 * my + r22 * mz; // column 2
        // 2 innovation measured - predicted (3 vector)
  
        let y = MagMeasVec::new(z[0] - hx0, z[1] - hx1, z[2] - hx2);
 
       let y01 = y[0];
       let y02 = y[1];
       let y03 = y[2];
       let z_mag = ((z[0] * z[0] + z[1] * z[1] + z[2] * z[2]).sqrt()) as f32;
        

      // 3 measurment Jacobian H (3x15)
      // h(x) depends only on oreintation -> only the orientation
      // is non-zero that block is the skew-symmetric matrix of the predicted
      // measurment vector h(x) = (hx0, hx1, hx2)
      let mut h = MagJacobian::zeros();

      // skew ([hx0, hx1, hx2])
      h[(0,6)] = 0.0;  h[(0,7)] = -hx2;  h[(0,8)] = hx1;
       h[(1,6)] = hx2;  h[(1,7)] = 0.0;  h[(1,8)] = -hx0;
        h[(2,6)] = -hx1;  h[(2,7)] = hx0;  h[(2,8)] = 0.0;
       
// 4 innovation covariance
let s: MagMeasMat = h * self.p * h.transpose() + self.R_mag;



// kalman gain equation 5 15x3 matrix
let s_inv = s.try_inverse().expect("mag not invertible");
let k: MagkalmanGain = self.p * h.transpose() * s_inv;


// 6a error state dx = K * y (15x1)
let dx: ErrorState = k * y;


// injection of error state into nominal
// Position
self.x[0] += dx[0];
self.x[1] += dx[1];
self.x[2] += dx[2];
 
 // velocity
 self.x[3] += dx[3];
self.x[4] += dx[4];
self.x[5] += dx[5];

// orientation error = small quartenion q * dq (* = quartenion addition or the weird (+) symbol)
let dth_x = dx[6];
let dth_y = dx[7];
let dth_z = dx[8];
let dth_norm = ((dth_x * dth_x + dth_y * dth_y + dth_z * dth_z).sqrt()) as f32;

let (dq0, dq1, dq2, dq3) = if dth_norm > 1e-6 {
    let s_half = ((dth_norm * 0.5).sin()) as f32 / dth_norm;
    (
        ((dth_norm * 0.5).cos()) as f32,
        dth_x * s_half,
        dth_y * s_half,
        dth_z * s_half,
    )
} else {
    // small angle approx if angle smaller than 1e-6
    (1.0, dth_x * 0.5, dth_y * 0.5, dth_z * 0.5)
};

let q0 = self.x[6];
let q1 = self.x[7];
let q2 = self.x[8];
let q3 = self.x[9];

// q_new = q * dq -- local convention which baro update step and predict both also follow
let new_q0:f32 = q0 * dq0 - q1 * dq1 - q2 * dq2 - q3 * dq3;
let new_q1:f32 = q0 * dq1 + q1 * dq0 + q2 * dq3 - q3 * dq2;
let new_q2:f32 = q0 * dq2 - q1 * dq3 + q2 * dq0 + q3 * dq1;
let new_q3:f32 = q0 * dq3 + q1 * dq2 - q2 * dq1 + q3 * dq0;
        
let norm: f32 =
((new_q0 * new_q0 + new_q1 * new_q1 + new_q2 * new_q2 + new_q3 * new_q3).sqrt()) as f32;
self.x[6] = new_q0 / norm;      
self.x[7] = new_q1 / norm;
self.x[8] = new_q2 / norm; 
self.x[9] = new_q3 / norm;     


// biases error state and nominal state have 1D difference so index are slightly different
self.x[10] += dx[9];
self.x[11] += dx[10];
self.x[12] += dx[11];
self.x[13] += dx[12];
self.x[14] += dx[13];
self.x[15] += dx[14];


// covariance update (joseph form like in the KF implementation)
         let i = ErrorCov::identity();
         let ikh = i - k * h;
         self.p = ikh * self.p * ikh.transpose() + k * self.R_mag * k.transpose();


// error state reset equation 7
// dx injected into nominal state; recalibrate P around the new nominal
 let mut g = ErrorCov::identity();
 g[(6,7)] = 0.5 * dth_z;
 g[(6,8)] = -0.5 * dth_y;
 g[(7,6)] = -0.5 * dth_z;
 g[(7,8)] = 0.5 * dth_x;
 g[(8,6)] = 0.5 * dth_y;
 g[(8,7)] = -0.5 * dth_x;
self.p = g * self.p * g.transpose();

let _dx = ErrorState::zeros();
        }
        
       pub fn update_gps(&mut self, z : GpsMeasVec) {
       // equation 1 and 2 in one step 
        let mut y = GpsMeasVec::zeros();
        y[(0,0)] =  z[0] - self.x[0]; // p-east
        y[(1,0)] =  z[1] - self.x[1]; // p-north
        y[(2,0)] = z[2] - self.x[3]; // v-east
        y[(3,0)] = z[3] - self.x[4]; // v-north
 
        let mut h = GpsJacobian::zeros();
        h[(0,0)] = 1.0; // pe
        h[(1,1)] = 1.0; // pn
        h[(2,3)] = 1.0; // ve
        h[(3,4)] = 1.0; // vn

        // innovation covariance
        let s = h * self.p * h.transpose() + self.R_gps;

       // kalman gain
       let s_inv = match s.try_inverse() {
        Some(inv) => inv,
        None => return,
       };
       // kalman gain
       let k: GpsKalmanGain = self.p * h.transpose() * s_inv;
   
   // error state correction
       let dx = k * y;
  // position     
self.x[0] += dx[0];
self.x[1] += dx[1];
self.x[2] += dx[2];
 
 // velocity
self.x[3] += dx[3];
self.x[4] += dx[4];
self.x[5] += dx[5];

// orientation error = small quartenion q * dq (* = quartenion addition or the weird (+) symbol)
let dth_x = dx[6];
let dth_y = dx[7];
let dth_z = dx[8];
let dth_norm = ((dth_x * dth_x + dth_y * dth_y + dth_z * dth_z).sqrt()) as f32;

let (dq0, dq1, dq2, dq3) = if dth_norm > 1e-6 {
    let s_half = ((dth_norm * 0.5).sin()) as f32 / dth_norm;
    (
        ((dth_norm * 0.5).cos()) as f32,
        dth_x * s_half,
        dth_y * s_half,
        dth_z * s_half,
    )
} else {
    // small angle approx if angle smaller than 1e-6
    (1.0, dth_x * 0.5, dth_y * 0.5, dth_z * 0.5)
};

let q0 = self.x[6];
let q1 = self.x[7];
let q2 = self.x[8];
let q3 = self.x[9];

// q_new = q * dq  local convention which baro update step and predict both also follow
let new_q0:f32 = q0 * dq0 - q1 * dq1 - q2 * dq2 - q3 * dq3;
let new_q1:f32 = q0 * dq1 + q1 * dq0 + q2 * dq3 - q3 * dq2;
let new_q2:f32 = q0 * dq2 - q1 * dq3 + q2 * dq0 + q3 * dq1;
let new_q3:f32 = q0 * dq3 + q1 * dq2 - q2 * dq1 + q3 * dq0;
        
let norm: f32 =
((new_q0 * new_q0 + new_q1 * new_q1 + new_q2 * new_q2 + new_q3 * new_q3).sqrt()) as f32;
self.x[6] = new_q0 / norm;      
self.x[7] = new_q1 / norm;
self.x[8] = new_q2 / norm; 
self.x[9] = new_q3 / norm;     


// biases error state and nominal state have 1D difference so index are slightly different
self.x[10] += dx[9];
self.x[11] += dx[10];
self.x[12] += dx[11];
self.x[13] += dx[12];
self.x[14] += dx[13];
self.x[15] += dx[14];


// covariance update (joseph form)
         let i = ErrorCov::identity();
         let ikh = i - k * h;
         self.p = ikh * self.p * ikh.transpose() + k * self.R_gps * k.transpose();


let mut g_reset = ErrorCov::identity();
// oreination covariance recalibration 
g_reset[(6,7)] = 0.5 * dth_z;
g_reset[(6,8)] = -0.5 * dth_y;
g_reset[(7,6)] = -0.5 * dth_z;
g_reset[(7,8)] = 0.5 * dth_x;
g_reset[(8,6)] = 0.5 * dth_y;
g_reset[(8,7)] = -0.5 * dth_x;
// resets error state to 0.
 self.p = g_reset * self.p * g_reset.transpose();
 let _dx = ErrorState::zeros();

       }


}




fn main() {
  let mut eskf = ESKF::new();
  let z = GpsMeasVec::from([10.0,0.0,0.0,0.0]);
  
  eskf.update_gps(z);
println!("{:?}", eskf);

}









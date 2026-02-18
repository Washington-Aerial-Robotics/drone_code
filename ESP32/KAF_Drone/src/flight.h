#ifndef FLIGHT_H
#define FLIGHT_H

#include "kaf_drone.h"


#define ESTIMATION_ENABLE   0b1000
#define NULL_MODE           0b0000
#define CALIBRATION_MODE    0b0001
#define INACTIVE_MODE       0b0010
#define MOTOR_SETPOINT_MODE 0b0011
#define ACCEL_SETPOINT_MODE 0b1001
#define VEL_SETPOINT_MODE   0b1010
#define POS_SETPOINT_MODE   0b1011
#define TRAJECTORY_MODE     0b1100

struct sensors {//FLIGHT DATA, calibrated sensor data in body frame
  float timeStep = 0;
  coordinate accelInput = { 0, 0, 0 };          // m/s^2
  coordinate gyroInput = { 0, 0, 0 };           // rad / s
  coordinate magInput = { 0, 0, 0 };            // 
  coordinate gpsInput = { 0, 0, 0 };            //
  coordinate baroInput = { 0, 0, 0 };           //
  bool accelUpdate = false;
  bool gyroUpdate = false;
  bool magUpdate = false;
  bool gpsUpdate = false;
  bool baroUpdate = false;
};

// class KalmanFilter3D; // May be needed? not sure


// does this transform body frame to world frame? why have dt?
void flight_attitudeEstimate( const coordinate ab, const coordinate wb, const float dt );

void flight_attitudeControl( const float ax, const float ay, const float az, const float tk, const float dt );

// need to implement
void flight_fullStateEstimation( const sensors* sensor );

memory flight_reset();
void flight_step( const sensors* sensor );

#endif


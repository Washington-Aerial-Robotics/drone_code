#include "kaf_drone.h"
#include "KalmanFilter3D.h"

#define ESTIMATION_ENABLE   0b1000
#define NULL_MODE           0b0000
#define CALIBRATION_MODE    0b0001
#define INACTIVE_MODE       0b0010
#define MOTOR_SETPOINT_MODE 0b0011
#define ACCEL_SETPOINT_MODE 0b1001
#define VEL_SETPOINT_MODE   0b1010
#define POS_SETPOINT_MODE   0b1011
#define TRAJECTORY_MODE     0b1100



struct sensors {//FLIGHT DATA, calibrated sensor data
  float timeStep = 0;
  coordinate accelInput = { 0, 0, 0 };          //
  coordinate gyroInput = { 0, 0, 0 };           //
  coordinate magInput = { 0, 0, 0 };            //
  coordinate gpsInput = { 0, 0, 0 };            //
  coordinate baroInput = { 0, 0, 0 };           //
  bool accelUpdate = false;
  bool gyroUpdate = false;
  bool magUpdate = false;
  bool gpsUpdate = false;
  bool baroUpdate = false;
};


KalmanFilter3D kf(0.01); // dt = 0.01s for 100 hz, adjust as required, not currently used anyways
unsigned long lastTime = 0;
unsigned long currentTime = micros();  // time in microseconds for dynamic dt values
float dt;



static float GyroMeasError = PI * (40.0f / 180.0f);     // gyroscope measurement error in rads/s (start at 40 deg/s)
static float GyroMeasDrift = PI * (0.0f / 180.0f);      // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
static float beta = sqrt(3.0f / 4.0f) * GyroMeasError;  // compute beta
static float zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift;  // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value

void flight_attitudeEstimate( const coordinate ab, const coordinate wb, const float dt );

void flight_attitudeControl( const float ax, const float ay, const float az, const float tk, const float dt );

// This function (should hopefully) take in calibrated sensors in the body frame
// and update the state quaternion, position, velocity, and angular velocity
void flight_fullStateEstimation( const sensors* sensor ) {
    float an = -sensor->accelInput[0];
    float ae = +sensor->accelInput[1];
    float ad = +sensor->accelInput[2];
    float gn = +sensor->gyroInput[0] * DEG_TO_RAD;
    float ge = -sensor->gyroInput[1] * DEG_TO_RAD;
    float gd = -sensor->gyroInput[2] * DEG_TO_RAD;
    float mn = +sensor->magInput[1];
    float me = -sensor->magInput[0];
    float md = +sensor->magInput[2];

    dt = (currentTime - lastTime) / 1e6; // convert to seconds
    lastTime = currentTime;
    currentTime = micros(); // time in microseconds since boot
    
    // updates attitude quaternion
    madgewick(an, ae, ad, gn, ge, gd, mn, me, md, kafenv.state.q, dt); // kafenv.q may be incorrect
    

    // 1. Process noise, should be tuned later
    const double q_noise = 0.01;

    // 2. Predict next state
    kf.predict(q_noise, dt);

    // 3. Position measurements (if available)
    double px = sensor->gpsUpdate ? sensor->gpsInput[0] : kf.x.data[0];
    double py = sensor->gpsUpdate ? sensor->gpsInput[1] : kf.x.data[3];
    double pz = sensor->baroUpdate ? sensor->baroInput[0] : kf.x.data[6];

    // rotate accelerometer to body frame
    float bodyFrameAcc[3] = {-an, ae, ad};
    float worldFrameAcc[3] = {0, 0, 0};
    rotateVectorByQuat(kafenv.state.q, bodyFrameAcc, worldFrameAcc);

    // subtract gravity, may need calibration
    worldFrameAcc[2] -= 9.807f;

    // 4. Acceleration measurements (from IMU)
    double ax = sensor->accelUpdate ? worldFrameAcc[0] : kf.x.data[2];
    double ay = sensor->accelUpdate ? worldFrameAcc[1] : kf.x.data[5];
    double az = sensor->accelUpdate ? worldFrameAcc[2] : kf.x.data[8];


    // 5. Measurement variances (tune, but for now assume it is given in sensors->..stdev)
    //double var_p = 0.5; // GPS uncertainty, test with different values
    //double var_a = 0.02; // Accelerometer uncertainty, test with different values
    //double var_b = 0.1; // barometer uncertainty, test with different values, not yet fully implemented

    // 6. Update the filter with measurements
    
    //kf.update(px, py, pz, ax, ay, az, var_p, var_a);
    var_pxy = sensor->gpsInput.stdev*sensor->gpsInput.stdev;
    var_pz = sensor->baroInput.stdev*sensor->baroInput.stdev;
    var_acc = sensor->accelInput.stdev* sensor->accelInput.stdev;

    kf.update(px, py, pz, ax, ay, az, var_pxy, var_pz, var_acc);

    // 7. Write estimated state back to global kafenv
    kafenv.state.x[0] = kf.x.data[0]; // position x
    kafenv.state.v[0] = kf.x.data[1]; // velocity x

    kafenv.state.x[1] = kf.x.data[3]; // position y
    kafenv.state.v[1] = kf.x.data[4]; // velocity y

    kafenv.state.x[2] = kf.x.data[6]; // position z
    kafenv.state.v[2] = kf.x.data[7]; // velocity z

    // Filtered angular velocity reduces to taking a rolling average
    // as there are no other inputs but the gyrometer
    double new_w_var = sensor->gyroInput.stdev * sensor->gyroInput.stdev;
    double old_w_var = kafenv.state.w.stdev * kafenv.state.w.stdev;
    double combined_w_var = 1 / (1/new_w_var + 1 / old_w_var);

    kafenv.w.stdev = sqrt( combined_w_var );
    
    kafenv.state.w.x = (sensor->gyroInput.x / new_w_var + kafenv.state.w.x / old_w_var) / (1/new_w_var + 1 / old_w_var);
    kafenv.state.w.y = (sensor->gyroInput.y / new_w_var + kafenv.state.w.y / old_w_var) / (1/new_w_var + 1 / old_w_var);
    kafenv.state.w.z = (sensor->gyroInput.z / new_w_var + kafenv.state.w.z / old_w_var) / (1/new_w_var + 1 / old_w_var);
}

memory flight_reset();
void flight_step( const sensors* sensor );


void madgwick(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float* q, float deltaT) {

    
    
    double q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];  // short name local variable for readability
    double recipNorm;
    double s0, s1, s2, s3;
    double qDot1, qDot2, qDot3, qDot4;
    double hx, hy;
    double _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

    // Rate of change of quaternion from gyroscope
    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

    // Normalise accelerometer measurement
    double a_norm = ax * ax + ay * ay + az * az;
    if (a_norm == 0.) return;  // handle NaN
    recipNorm = 1.0 / sqrt(a_norm);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Normalise magnetometer measurement
    double m_norm = mx * mx + my * my + mz * mz;
    if (m_norm == 0.) return;  // handle NaN
    recipNorm = 1.0 / sqrt(m_norm);
    mx *= recipNorm;
    my *= recipNorm;
    mz *= recipNorm;

    // Auxiliary variables to avoid repeated arithmetic
    _2q0mx = 2.0f * q0 * mx;
    _2q0my = 2.0f * q0 * my;
    _2q0mz = 2.0f * q0 * mz;
    _2q1mx = 2.0f * q1 * mx;
    _2q0 = 2.0f * q0;
    _2q1 = 2.0f * q1;
    _2q2 = 2.0f * q2;
    _2q3 = 2.0f * q3;
    _2q0q2 = 2.0f * q0 * q2;
    _2q2q3 = 2.0f * q2 * q3;
    q0q0 = q0 * q0;
    q0q1 = q0 * q1;
    q0q2 = q0 * q2;
    q0q3 = q0 * q3;
    q1q1 = q1 * q1;
    q1q2 = q1 * q2;
    q1q3 = q1 * q3;
    q2q2 = q2 * q2;
    q2q3 = q2 * q3;
    q3q3 = q3 * q3;

    // Reference direction of Earth's magnetic field
    hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
    hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
    _2bx = sqrt(hx * hx + hy * hy);
    _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
    _4bx = 2.0f * _2bx;
    _4bz = 2.0f * _2bz;

    // Gradient decent algorithm corrective step
    s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    recipNorm = 1.0 / sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);  // normalise step magnitude
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    // Apply feedback step
    qDot1 -= beta * s0;
    qDot2 -= beta * s1;
    qDot3 -= beta * s2;
    qDot4 -= beta * s3;

    // Integrate rate of change of quaternion to yield quaternion
    q0 += qDot1 * deltaT;
    q1 += qDot2 * deltaT;
    q2 += qDot3 * deltaT;
    q3 += qDot4 * deltaT;

    // Normalise quaternion
    recipNorm = 1.0 / sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;

    q[0] = q0;
    q[1] = q1;
    q[2] = q2;
    q[3] = q3;
}


inline void rotateVectorByQuat(
        const float q[4],     // {w, x, y, z}
        const float v[3],     // input vector (body)
        float out[3]          // rotated vector (world)
    ) {
        const float w = q[0];
        const float x = q[1];
        const float y = q[2];
        const float z = q[3];

        // t = 2 * cross(q_vec, v)
        const float tx = 2.0f * (y * v[2] - z * v[1]);
        const float ty = 2.0f * (z * v[0] - x * v[2]);
        const float tz = 2.0f * (x * v[1] - y * v[0]);

        // v' = v + w * t + cross(q_vec, t)
        out[0] = v[0] + w * tx + (y * tz - z * ty);
        out[1] = v[1] + w * ty + (z * tx - x * tz);
        out[2] = v[2] + w * tz + (x * ty - y * tx);
    }
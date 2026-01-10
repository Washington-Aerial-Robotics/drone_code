#pragma once
#ifndef KALMANFILTER_H
#define KALMANFILTER_H

#include <cstdint>
#include <cmath>
#include <Arduino.h>

// Simple 3-state Kalman filter per axis: [pos; vel; acc_bias]
// Discrete-time model (dt seconds):
// pos_{k+1}  = pos_k + vel_k*dt + 0.5*a_meas*dt^2 - 0.5*bias_k*dt^2
// vel_{k+1}  = vel_k + a_meas*dt - bias_k*dt
// bias_{k+1} = bias_k  (random walk)

// Note: a_meas is the accelerometer measurement along that axis (m/s^2)

struct KFState {
    float Pos[3]; // in meters, world frame, NED
    float Vel[3]; // in m/s, world frame, NED
    float Acc[3]; // estimated acceleration (filtered), in m/s^2, world frame, NED

    float Ang_Vel[3]; // angular velocity, in deg/s, body frame XYZ
    float Ang_Acc[3]; // angular acc, in deg/s, body frame XYZ

    double deltaT{0.0};
    uint32_t newTime{0}, oldTime{0};
};

class KalmanFilter {
public:
    KalmanFilter()
        : state_{}   // zero-initialize
    {
        initDefaults();
    }

    // Public getters
    const float* Pos() const { return state_.Pos; }
    const float* Vel() const { return state_.Vel; }
    const float* Acc() const { return state_.Acc; }

    const float* AngVel() const { return state_.Ang_Vel; }
    const float* AngAcc() const { return state_.Ang_Acc; }


    // Update internal timing; should be called at the start of a predict step
    void update_time() {
        state_.newTime = micros();
        if (state_.oldTime == 0u) {
            // first call: initialize and avoid large dt
            state_.deltaT = 0.0;
        } else {
            uint32_t diff = state_.newTime - state_.oldTime;
            state_.deltaT = static_cast<double>(diff) * 1e-6; // micros -> seconds
            if (state_.deltaT <= 0) state_.deltaT = 1e-6;
        }
        state_.oldTime = state_.newTime;
    }

    // Predict using accelerometer measurement in m/s^2 for each axis (NED)
    // This performs the predict step for each axis separately.
    void predict(const float acc_m[3]) {
        update_time();
        double dt = state_.deltaT;
        if (dt <= 0) return;

        // For each axis, run predict
        for (int axis = 0; axis < 3; ++axis) {
            predictAxis(axis, static_cast<double>(acc_m[axis]), dt);
            // store a filtered estimate of acceleration (acc_meas - bias_est)
            state_.Acc[axis] = static_cast<float>( acc_m[axis] - x_[axis][2] );
        }

        // note: angular rates/acc not fused here
    }

    // Update from a position measurement (m) for each axis. Provide NaN for axes without measurement.
    // Example: for GPS that only gives horizontal position, pass z as NAN if unavailable.
    void updatePosition(const float pos_meas[3]) {
        for (int axis = 0; axis < 3; ++axis) {
            if (!std::isnan(pos_meas[axis])) {
                updatePositionAxis(axis, static_cast<double>(pos_meas[axis]));
                // copy filtered states back to KFState storage for easy access
                state_.Pos[axis] = static_cast<float>( x_[axis][0] );
                state_.Vel[axis] = static_cast<float>( x_[axis][1] );
            }
        }
    }

    // Convenience update for a single axis
    void updatePositionAxisSingle(int axis, float pos_meas) {
        if (axis < 0 || axis > 2) return;
        updatePositionAxis(axis, static_cast<double>(pos_meas));
        state_.Pos[axis] = static_cast<float>( x_[axis][0] );
        state_.Vel[axis] = static_cast<float>( x_[axis][1] );
    }

    // Set process / measurement noise parameters (tuning)
    // sigma_a: std dev of accelerometer noise (m/s^2)
    // sigma_b: std dev of bias random walk per second (m/s^2 / sqrt(s))
    // sigma_pos: std dev of position measurement (m)
    void setNoiseParams(float sigma_a, float sigma_bias, float sigma_pos) {
        sigma_acc_ = sigma_a;
        sigma_bias_rw_ = sigma_bias;
        R_pos_ = sigma_pos * sigma_pos;
    }

    // Initialize state with known position/vel
    void initState(const float pos[3], const float vel[3]) {
        for (int i = 0; i < 3; ++i) {
            x_[i][0] = pos[i];
            x_[i][1] = vel[i];
            x_[i][2] = 0.0; // bias initial guess
            // initialize covariance fairly large
            for (int r = 0; r < 3; ++r) {
                for (int c = 0; c < 3; ++c) {
                    P_[i][r][c] = 0.0f;
                }
            }
            P_[i][0][0] = 5.0f;  // pos var (m^2)
            P_[i][1][1] = 1.0f;  // vel var ((m/s)^2)
            P_[i][2][2] = 0.1f;  // bias var ((m/s^2)^2)
            state_.Pos[i] = pos[i];
            state_.Vel[i] = vel[i];
        }
    }

    const float* getPos() const { return state_.Pos; }
    const float* getVel() const { return state_.Vel; }
    const float* getAcc() const { return state_.Acc; }
    const float* getAngVel() const { return state_.Ang_Vel; }
    const float* getAng_Acc() const { return state_.Ang_Acc; }

private:
    KFState state_;

    // Per-axis states: x = [pos; vel; bias]
    // x_[axis][i]
    double x_[3][3]{};      // per-axis state vector in double precision internally
    float P_[3][3][3]{};    // per-axis covariance matrices (float to save memory)

    // Noise parameters (tunable)
    double sigma_acc_ = 0.2;      // m/s^2 (accelerometer measurement noise std)
    double sigma_bias_rw_ = 0.001; // m/s^2 per sqrt(sec) (bias random walk)
    double R_pos_ = 5.0;          // position measurement variance (m^2)

    // helper: small numerical regularization
    const double eps_ = 1e-9;

    // initialize default covariances and state
    void initDefaults() {
        // default zero state already from constructor; set a reasonable P
        for (int axis = 0; axis < 3; ++axis) {
            x_[axis][0] = 0.0;
            x_[axis][1] = 0.0;
            x_[axis][2] = 0.0;
            // Covariances
            P_[axis][0][0] = 10.0f;  // pos variance
            P_[axis][1][1] = 4.0f;   // vel variance
            P_[axis][2][2] = 0.5f;   // bias variance
            P_[axis][0][1] = P_[axis][1][0] = 0.0f;
            P_[axis][0][2] = P_[axis][2][0] = 0.0f;
            P_[axis][1][2] = P_[axis][2][1] = 0.0f;
        }
    }

    // Predict for one axis
    void predictAxis(int axis, double a_meas, double dt) {
        const double acc_deadband = 0.05; // m/s^2, adjust to your sensor noise

        // apply deadband for motion
        double a_for_motion = (fabs(a_meas) < acc_deadband) ? 0.0 : a_meas;

        // build F and B for this dt
        double dt2 = dt*dt;
        double F00 = 1.0;
        double F01 = dt;
        double F02 = -0.5*dt2;   // effect of bias on pos
        double F10 = 0.0;
        double F11 = 1.0;
        double F12 = -dt;        // effect of bias on vel
        double F20 = 0.0;
        double F21 = 0.0;
        double F22 = 1.0;

        double b0 = 0.5*dt2;
        double b1 = dt;
        double b2 = 0.0;

        // predict state x = F*x + B*a_for_motion
        double x0 = F00*x_[axis][0] + F01*x_[axis][1] + F02*x_[axis][2] + b0*a_for_motion;
        double x1 = F10*x_[axis][0] + F11*x_[axis][1] + F12*x_[axis][2] + b1*a_for_motion;
        double x2 = F20*x_[axis][0] + F21*x_[axis][1] + F22*x_[axis][2] + b2*a_meas; // full a_meas for bias correction

        x_[axis][0] = x0;
        x_[axis][1] = x1;
        x_[axis][2] = x2;

        // store filtered acceleration for reference
        state_.Acc[axis] = static_cast<float>(a_for_motion);

        // --- Predict covariance P = F*P*F^T + Q ---
        float FPmat[3][3];
        FPmat[0][0] = static_cast<float>(F00*P_[axis][0][0] + F01*P_[axis][1][0] + F02*P_[axis][2][0]);
        FPmat[0][1] = static_cast<float>(F00*P_[axis][0][1] + F01*P_[axis][1][1] + F02*P_[axis][2][1]);
        FPmat[0][2] = static_cast<float>(F00*P_[axis][0][2] + F01*P_[axis][1][2] + F02*P_[axis][2][2]);

        FPmat[1][0] = static_cast<float>(F10*P_[axis][0][0] + F11*P_[axis][1][0] + F12*P_[axis][2][0]);
        FPmat[1][1] = static_cast<float>(F10*P_[axis][0][1] + F11*P_[axis][1][1] + F12*P_[axis][2][1]);
        FPmat[1][2] = static_cast<float>(F10*P_[axis][0][2] + F11*P_[axis][1][2] + F12*P_[axis][2][2]);

        FPmat[2][0] = static_cast<float>(F20*P_[axis][0][0] + F21*P_[axis][1][0] + F22*P_[axis][2][0]);
        FPmat[2][1] = static_cast<float>(F20*P_[axis][0][1] + F21*P_[axis][1][1] + F22*P_[axis][2][1]);
        FPmat[2][2] = static_cast<float>(F20*P_[axis][0][2] + F21*P_[axis][1][2] + F22*P_[axis][2][2]);

        // Ppred = FP * F^T
        float Pnew[3][3];
        Pnew[0][0] = FPmat[0][0]*F00 + FPmat[0][1]*F01 + FPmat[0][2]*F02;
        Pnew[0][1] = FPmat[0][0]*F10 + FPmat[0][1]*F11 + FPmat[0][2]*F12;
        Pnew[0][2] = FPmat[0][0]*F20 + FPmat[0][1]*F21 + FPmat[0][2]*F22;

        Pnew[1][0] = FPmat[1][0]*F00 + FPmat[1][1]*F01 + FPmat[1][2]*F02;
        Pnew[1][1] = FPmat[1][0]*F10 + FPmat[1][1]*F11 + FPmat[1][2]*F12;
        Pnew[1][2] = FPmat[1][0]*F20 + FPmat[1][1]*F21 + FPmat[1][2]*F22;

        Pnew[2][0] = FPmat[2][0]*F00 + FPmat[2][1]*F01 + FPmat[2][2]*F02;
        Pnew[2][1] = FPmat[2][0]*F10 + FPmat[2][1]*F11 + FPmat[2][2]*F12;
        Pnew[2][2] = FPmat[2][0]*F20 + FPmat[2][1]*F21 + FPmat[2][2]*F22;

        // Add process noise (same as before)
        double G0 = b0;
        double G1 = b1;
        double G2 = 0.0;

        double q00 = (G0*G0) * (sigma_acc_*sigma_acc_);
        double q01 = (G0*G1) * (sigma_acc_*sigma_acc_);
        double q02 = (G0*G2) * (sigma_acc_*sigma_acc_);
        double q11 = (G1*G1) * (sigma_acc_*sigma_acc_);
        double q12 = (G1*G2) * (sigma_acc_*sigma_acc_);
        double q22 = (G2*G2) * (sigma_acc_*sigma_acc_);
        double q_bias = (sigma_bias_rw_ * sigma_bias_rw_) * dt;

        Pnew[0][0] += static_cast<float>(q00 + eps_);
        Pnew[0][1] += static_cast<float>(q01);
        Pnew[0][2] += static_cast<float>(q02);
        Pnew[1][0] += static_cast<float>(q01);
        Pnew[1][1] += static_cast<float>(q11 + eps_);
        Pnew[1][2] += static_cast<float>(q12);
        Pnew[2][0] += static_cast<float>(q02);
        Pnew[2][1] += static_cast<float>(q12);
        Pnew[2][2] += static_cast<float>(q22 + q_bias + eps_);

        // store Pnew into P_
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c)
                P_[axis][r][c] = Pnew[r][c];
    }


    // Update position measurement for one axis
    void updatePositionAxis(int axis, double z_pos) {
        // H = [1 0 0], measurement is scalar
        // Innovation covariance S = H * P * H^T + R = P[0][0] + R_pos_
        double P00 = P_[axis][0][0];
        double S = static_cast<double>(P00) + R_pos_;
        if (S <= 0) S = eps_;

        // Kalman gain K = P * H^T / S = first column of P divided by S
        double K0 = P_[axis][0][0] / S;
        double K1 = P_[axis][1][0] / S;
        double K2 = P_[axis][2][0] / S;

        // innovation y = z - H*x = z - x[0]
        double y = z_pos - x_[axis][0];

        // state update x = x + K*y
        x_[axis][0] += K0 * y;
        x_[axis][1] += K1 * y;
        x_[axis][2] += K2 * y;

        // covariance update P = (I - K*H) * P
        // (I - K*H) = [[1-K0, 0, 0], [-K1, 1, 0], [-K2, 0, 1]]
        float Pnew[3][3];
        // row 0
        Pnew[0][0] = static_cast<float>((1.0 - K0) * P_[axis][0][0]);
        Pnew[0][1] = static_cast<float>((1.0 - K0) * P_[axis][0][1]);
        Pnew[0][2] = static_cast<float>((1.0 - K0) * P_[axis][0][2]);
        // row 1
        Pnew[1][0] = static_cast<float>(-K1 * P_[axis][0][0] + P_[axis][1][0]);
        Pnew[1][1] = static_cast<float>(-K1 * P_[axis][0][1] + P_[axis][1][1]);
        Pnew[1][2] = static_cast<float>(-K1 * P_[axis][0][2] + P_[axis][1][2]);
        // row 2
        Pnew[2][0] = static_cast<float>(-K2 * P_[axis][0][0] + P_[axis][2][0]);
        Pnew[2][1] = static_cast<float>(-K2 * P_[axis][0][1] + P_[axis][2][1]);
        Pnew[2][2] = static_cast<float>(-K2 * P_[axis][0][2] + P_[axis][2][2]);

        // enforce symmetry (average with transpose)
        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 3; ++c) {
                P_[axis][r][c] = 0.5f * (Pnew[r][c] + Pnew[c][r]);
            }
        }
    }
};

#endif
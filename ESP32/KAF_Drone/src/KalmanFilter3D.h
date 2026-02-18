#include <iostream>
#include <vector>

struct Matrix9x9 {
    double data[9][9] = {0};
    // Helper for Identity
    static Matrix9x9 Identity() {
        Matrix9x9 m;
        for(int i=0; i<9; ++i) m.data[i][i] = 1.0;
        return m;
    }
};

struct Vector9 {
    double data[9] = {0};
};

class KalmanFilter3D {
public:
    Vector9 x;     // State: [px, vx, ax, py, vy, ay, pz, vz, az]
    Matrix9x9 P;   // Covariance
    double dt;

    KalmanFilter3D(double time_step) : dt(time_step) {
        P = Matrix9x9::Identity();
        // Initialize with high uncertainty for velocity/acceleration
        for(int i=0; i<9; ++i) P.data[i][i] = 10.0; 
    }

    // Prediction Step (State Transition)
    void predict(double q_noise, double dt_override = -1) {
        if (dt_override > 0) {
            dt = dt_override;
        }
        double dt2 = 0.5 * dt * dt;
        
        // 1. Predict State: x = F * x
        Vector9 next_x;
        for (int i = 0; i < 3; ++i) { // For each axis x, y, z
            int offset = i * 3;
            next_x.data[offset]     = x.data[offset] + (dt * x.data[offset+1]) + (dt2 * x.data[offset+2]);
            next_x.data[offset+1]   = x.data[offset+1] + (dt * x.data[offset+2]);
            next_x.data[offset+2]   = x.data[offset+2];
        }
        x = next_x;

        // 2. Predict Covariance: P = FPF' + Q (Simplified diagonal Q)
        // apply kinematic uncertainty
        for(int i=0; i<9; ++i) P.data[i][i] += q_noise;
    }

    /**
     * Sequential Update: Updates the filter one measurement at a time.
     * This removes the need for a 6x6 Matrix Inverse.
     */
    void update_single(int state_index, double measurement, double variance) {
        // H is just a vector with 1 at state_index
        // Innovation: y = z - Hx
        double y = measurement - x.data[state_index];
        
        // Innovation Covariance: S = HPH' + R
        double s = P.data[state_index][state_index] + variance;
        
        // Kalman Gain: K = PH' / S
        double k[9];
        for(int i=0; i<9; ++i) {
            k[i] = P.data[i][state_index] / s;
        }

        // Update State: x = x + Ky
        for(int i=0; i<9; ++i) {
            x.data[i] += k[i] * y;
        }

        // Update Covariance: P = (I - KH)P
        for(int i=0; i<9; ++i) {
            for(int j=0; j<9; ++j) {
                P.data[i][j] -= k[i] * P.data[state_index][j];
            }
        }
    }

    void update(double px, double py, double pz, double ax, double ay, double az, 
                double var_pxy, double var_pz, double var_a) {
        // Position Updates
        update_single(0, px, var_pxy);
        update_single(3, py, var_pxy);
        update_single(6, pz, var_pz);
        // Acceleration Updates
        update_single(2, ax, var_a);
        update_single(5, ay, var_a);
        update_single(8, az, var_a);
    }
};

int main() {
    KalmanFilter3D kf(0.1); // 0.1s dt

    // Example: Moving along X axis
    kf.predict(0.01); // process noise
    kf.update(1.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.1, 0.02);

    std::cout << "Estimated X Pos: " << kf.x.data[0] << " Vel: " << kf.x.data[1] << std::endl;
    return 0;
}
#include <Arduino.h>
#include "MPU9250.h"
#include <cmath>

MPU9250 mpu;
int n = 0;

void print_calibration() {
    Serial.println("< calibration parameters >");
    Serial.println("accel bias [g]: ");
    Serial.print(mpu.getAccBiasX() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getAccBiasY() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getAccBiasZ() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.println();
    Serial.println("gyro bias [deg/s]: ");
    Serial.print(mpu.getGyroBiasX() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getGyroBiasY() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getGyroBiasZ() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.println();
    Serial.println("mag bias [mG]: ");
    Serial.print(mpu.getMagBiasX());
    Serial.print(", ");
    Serial.print(mpu.getMagBiasY());
    Serial.print(", ");
    Serial.print(mpu.getMagBiasZ());
    Serial.println();
    Serial.println("mag scale []: ");
    Serial.print(mpu.getMagScaleX());
    Serial.print(", ");
    Serial.print(mpu.getMagScaleY());
    Serial.print(", ");
    Serial.print(mpu.getMagScaleZ());
    Serial.println();
}

void setup() {
    Serial.begin(115200);
    Wire.begin();
    delay(2000);

    MPU9250Setting setting;
    setting.accel_fs_sel = ACCEL_FS_SEL::A16G;
    setting.gyro_fs_sel = GYRO_FS_SEL::G2000DPS;
    setting.mag_output_bits = MAG_OUTPUT_BITS::M16BITS;
    setting.fifo_sample_rate = FIFO_SAMPLE_RATE::SMPL_250HZ;
    setting.gyro_fchoice = 0x03;
    setting.gyro_dlpf_cfg = GYRO_DLPF_CFG::DLPF_92HZ;
    setting.accel_fchoice = 0x01;
    setting.accel_dlpf_cfg = ACCEL_DLPF_CFG::DLPF_99HZ;
    mpu.setMagneticDeclination(14);
    mpu.selectFilter(QuatFilterSel::MADGWICK);

    if (!mpu.setup(0x68, setting)) {  // change to your own address
        while (1) {
            Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
            delay(5000);
        }
    }

    //Serial.println("Accel Gyro calibration will start in 5sec.");
    //Serial.println("Please leave the device still on the flat plane.");
    //mpu.verbose(true);
    delay(200);
    //mpu.calibrateAccelGyro();
    //print_calibration();
    Serial.println("setup complete");

  }


void print_roll_pitch_yaw() {
    //Serial.print("Yaw, Pitch, Roll: ");
    Serial.print(mpu.getYaw(), 2);
    Serial.print(",");
    Serial.print(mpu.getPitch(), 2);
    Serial.print(",");
    Serial.println(mpu.getRoll(), 2);

}

void print_mag_scale() {
  Serial.print("Scale factors for mag: ");
  Serial.print(", ");
  Serial.print(mpu.getMagScaleX());
  Serial.print(", ");
  Serial.print(mpu.getMagScaleY());
  Serial.print(", ");
  Serial.print(mpu.getMagScaleZ());

}

double get_mag_magnitude() {
    double a = sqrt(mpu.getMagX()*mpu.getMagX()*0.1*0.1+mpu.getMagY()*mpu.getMagY()*0.1*0.1+mpu.getMagZ()*mpu.getMagZ()*0.1*0.1); // uT
    return a;
}

void loop() {
    if (mpu.update()) {
        n++;
        if (n%10 == 1) {
            Serial.print(mpu.getQuaternionW(), 3);
            Serial.print(",");
            Serial.print(mpu.getQuaternionX(), 3);
            Serial.print(",");
            Serial.print(mpu.getQuaternionY(), 3);
            Serial.print(",");
            Serial.print(mpu.getQuaternionZ(), 3);
            Serial.print(",");
            Serial.print(mpu.getAccX(), 3);
            Serial.print(",");
            Serial.print(mpu.getAccY(), 3);
            Serial.print(",");
            Serial.print(mpu.getAccZ(), 3);
            Serial.print(",");
            Serial.print(mpu.getMagX(), 3);
            Serial.print(",");
            Serial.print(mpu.getMagY(), 3);
            Serial.print(",");
            Serial.println(mpu.getMagZ(), 3);
        }
    }
}




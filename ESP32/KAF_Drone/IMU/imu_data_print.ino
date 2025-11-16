#include "MPU9250.h"

MPU9250 mpu;

void setup() {
    Serial.begin(1000000);
    Wire.begin();
    delay(2000);

    if (!mpu.setup(0x68)) {  // change to your own address
        while (1) {
            Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
            delay(5000);
        }
    }

    // calibrate anytime you want to
    Serial.println("Accel Gyro calibration will start in 5sec.");
    Serial.println("Please leave the device still on the flat plane.");
    mpu.verbose(true);
    delay(5000);
    mpu.calibrateAccelGyro();

    Serial.println("Mag calibration will start in 5sec.");
    Serial.println("Please Wave device in a figure eight until done.");
    delay(5000);
    mpu.calibrateMag();

    //print_calibration();
    mpu.verbose(false);


    // Print CSV header
    Serial.println("Time(ms),Yaw,Pitch,Roll,AccX,AccY,AccZ,GyroX,GyroY,GyroZ,MagX,MagY,MagZ,LinearAccX,LinearAccY,LinearAccZ");
}

void loop() {
    if (mpu.update()) {
        static uint32_t prev_ms = millis();
        if (millis() > prev_ms + 8) {
            print_roll_pitch_csv();
            prev_ms = millis();
        }
    }
}

void print_roll_pitch_csv() {
    Serial.print(millis()); Serial.print(",");
    Serial.print(mpu.getYaw(), 2); Serial.print(",");
    Serial.print(mpu.getPitch(), 2); Serial.print(",");
    Serial.print(mpu.getRoll(), 2); Serial.print(",");
    Serial.print(mpu.getAccX(), 2); Serial.print(",");
    Serial.print(mpu.getAccY(), 2); Serial.print(",");
    Serial.print(mpu.getAccZ(), 2); Serial.print(",");
    Serial.print(mpu.getGyroX(), 2); Serial.print(",");
    Serial.print(mpu.getGyroY(), 2); Serial.print(",");
    Serial.print(mpu.getGyroZ(), 2); Serial.print(",");
    Serial.print(mpu.getMagX(), 2); Serial.print(",");
    Serial.print(mpu.getMagY(), 2); Serial.print(",");
    Serial.print(mpu.getMagZ(), 2); Serial.print(",");
    Serial.print(mpu.getLinearAccX(), 2); Serial.print(",");
    Serial.print(mpu.getLinearAccY(), 2); Serial.print(",");
    Serial.println(mpu.getLinearAccZ(), 2);
}

/*
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
}*/

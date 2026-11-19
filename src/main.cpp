/**
 * VÍ DỤ 4-2a: ĐỌC CẢM BIẾN QUÁN TÍNH 9 TRỤC (IMU) BẰNG I2C THUẦN
 */
#include <Arduino.h>
#include <PTITCube.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <QMC5883P.h>
#include <math.h>

PTIT_CDH cdh;

Adafruit_MPU6050 mpu;
QMC5883P qmc;

void setup() {
    cdh.begin(115200);
    Wire.begin(21, 22);

    mpu.begin(0x68);

    // Đánh thức MPU6500 (Ghi 0 vào thanh ghi PWR_MGMT_1)
    Wire.beginTransmission(0x68);
    Wire.write(0x6B);
    Wire.write(0x00);
    Wire.endTransmission();

    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);

    if (qmc.begin()) {
        qmc.setMode(QMC5883P_MODE_CONTINUOUS);
        qmc.setRange(QMC5883P_RANGE_2G);
        qmc.setODR(QMC5883P_ODR_50HZ);
        qmc.setOSR(QMC5883P_OSR_8);
        qmc.setDSR(QMC5883P_DSR_1);
        qmc.setSetResetMode(QMC5883P_SETRESET_ON);
    }
}

void loop() {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    float ax = a.acceleration.x;
    float ay = a.acceleration.y;
    float az = a.acceleration.z;

    // Tính góc nghiêng (Tilt Angle)
    float roll  = atan2(ay, az) * 180.0 / PI;
    float pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;

    // Đọc từ trường
    int16_t raw_mx, raw_my, raw_mz;
    qmc.readRawData(&raw_mx, &raw_my, &raw_mz);
    float mx = raw_mx * 0.1f;
    float my = raw_my * 0.1f;
    float mz = raw_mz * 0.1f;

    float heading = atan2(my, mx) * 180.0 / PI;
    if (heading < 0) heading += 360.0;

    cdh.printf("[IMU] Roll: %6.1f° | Pitch: %6.1f° | Heading: %5.1f° | Acc: (%5.1f, %5.1f, %5.1f)\n",
               roll, pitch, heading, ax, ay, az);

    delay(1000);
}

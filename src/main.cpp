/**
 * VÍ DỤ 4-2a: ĐỌC CẢM BIẾN QUÁN TÍNH 9 TRỤC (IMU) & LA BÀN BẰNG I2C THUẦN
 * 
 * Mô tả:
 * - Đọc trực tiếp thanh ghi MPU6500 qua I2C để lấy Gia tốc (Acc) & Vận tốc góc (Gyro).
 * - Tính toán Góc nghiêng vệ tinh (Tilt Angle: Roll & Pitch).
 * - Đọc cảm biến Từ trường QMC5883P (Magnetometer: X, Y, Z tính bằng uT).
 * - Tính toán góc hướng La bàn không gian (Compass Heading: 0 - 360°).
 */
#include <Arduino.h>
#include <PTITCube.h>
#include <Wire.h>
#include <Adafruit_QMC5883P.h>
#include <math.h>

PTIT_CDH cdh;

#define MPU_ADDR 0x68 // Địa chỉ I2C của MPU6500

Adafruit_QMC5883P qmc;
bool qmcReady = false;

void setup() {
    cdh.begin(115200);
    Wire.begin(21, 22);

    // 1. Khởi tạo MPU6500 qua thanh ghi I2C thuần
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x6B); // Thanh ghi PWR_MGMT_1
    Wire.write(0x00); // Đánh thức chip khỏi chế độ Sleep
    Wire.endTransmission();

    // 2. Khởi tạo cảm biến La bàn từ trường QMC5883P (0x2C hoặc 0x0D)
    if (qmc.begin(0x2C) || qmc.begin(0x0D)) {
        qmcReady = true;
        qmc.setMode(QMC5883P_MODE_CONTINUOUS);
        qmc.setRange(QMC5883P_RANGE_2G);
        qmc.setODR(QMC5883P_ODR_50HZ);
        qmc.setOSR(QMC5883P_OSR_8);
        qmc.setDSR(QMC5883P_DSR_1);
        qmc.setSetResetMode(QMC5883P_SETRESET_ON);
    }
}

void loop() {
    // 1. Đọc dữ liệu Gia tốc & Con quay từ MPU6500 (Burst read 14 bytes từ thanh ghi 0x3B)
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom((uint16_t)MPU_ADDR, (uint8_t)14, (bool)true);

    float ax = 0, ay = 0, az = 0;
    float gx = 0, gy = 0, gz = 0;

    if (Wire.available() >= 14) {
        int16_t raw_ax = (Wire.read() << 8) | Wire.read();
        int16_t raw_ay = (Wire.read() << 8) | Wire.read();
        int16_t raw_az = (Wire.read() << 8) | Wire.read();
        Wire.read(); Wire.read(); // Bỏ qua 2 bytes nhiệt độ nội chip
        int16_t raw_gx = (Wire.read() << 8) | Wire.read();
        int16_t raw_gy = (Wire.read() << 8) | Wire.read();
        int16_t raw_gz = (Wire.read() << 8) | Wire.read();

        // Chuyển đổi gia tốc sang m/s^2 (dải ±2g: 16384 LSB/g, 1g = 9.80665 m/s^2)
        ax = ((float)raw_ax / 16384.0f) * 9.80665f;
        ay = ((float)raw_ay / 16384.0f) * 9.80665f;
        az = ((float)raw_az / 16384.0f) * 9.80665f;

        // Chuyển đổi vận tốc góc sang rad/s (dải ±250 deg/s: 131 LSB/(deg/s))
        gx = ((float)raw_gx / 131.0f) * (PI / 180.0f);
        gy = ((float)raw_gy / 131.0f) * (PI / 180.0f);
        gz = ((float)raw_gz / 131.0f) * (PI / 180.0f);
    }

    // 2. Tính góc nghiêng (Roll & Pitch)
    float roll  = atan2(ay, az) * 180.0 / PI;
    float pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;

    // 3. Đọc dữ liệu từ trường La bàn (QMC5883P)
    float mx = 0, my = 0, mz = 0;
    if (qmcReady) {
        float qx, qy, qz;
        if (qmc.getGaussField(&qx, &qy, &qz)) {
            // 1 Gauss = 100 microTesla (uT)
            mx = qx * 100.0f;
            my = qy * 100.0f;
            mz = qz * 100.0f;
        }
    }

    // 4. Tính góc hướng La bàn không gian (Heading: 0 - 360°)
    float heading = atan2(my, mx) * 180.0 / PI;
    if (heading < 0) heading += 360.0;

    // 5. Hiển thị dữ liệu toàn diện 9 trục & La bàn ra Serial
    cdh.println("--- DỮ LIỆU CẢM BIẾN QUÁN TÍNH & LA BÀN (9-DOF IMU) ---");
    cdh.printf("  Gia tốc (m/s^2)  : X = %6.2f | Y = %6.2f | Z = %6.2f\n", ax, ay, az);
    cdh.printf("  Con quay (rad/s) : X = %6.2f | Y = %6.2f | Z = %6.2f\n", gx, gy, gz);
    cdh.printf("  Từ trường (uT)   : X = %6.2f | Y = %6.2f | Z = %6.2f\n", mx, my, mz);
    cdh.printf("  Góc hướng La bàn : %5.1f°\n", heading);
    cdh.printf("  Góc nghiêng      : Roll = %5.1f° | Pitch = %5.1f°\n\n", roll, pitch);

    delay(1000);
}

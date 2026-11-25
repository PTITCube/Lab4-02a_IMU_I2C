/**
 * VÍ DỤ 4-2a: ĐỌC CẢM BIẾN QUÁN TÍNH 9 TRỤC (IMU) & LA BÀN BẰNG I2C THUẦN
 */
#include <Arduino.h>
#include <PTITCube.h>
#include <Wire.h>
#include <Adafruit_QMC5883P.h>
#include <math.h>

PTIT_CDH cdh;
PTIT_EPS eps;

Adafruit_QMC5883P qmc;
uint8_t mpuAddr = 0x68;
bool mpuReady = false;
bool qmcReady = false;

bool initMPU(uint8_t addr) {
    Wire.beginTransmission(addr);
    Wire.write(0x75); // WHO_AM_I
    if (Wire.endTransmission() != 0) return false;

    Wire.requestFrom((uint8_t)addr, (uint8_t)1);
    if (!Wire.available()) return false;
    uint8_t whoami = Wire.read();
    if (whoami != 0x68 && whoami != 0x70 && whoami != 0x71 && whoami != 0x73) {
        return false;
    }

    // 1. Đánh thức MPU (Ghi 0 vào thanh ghi PWR_MGMT_1 - 0x6B)
    Wire.beginTransmission(addr);
    Wire.write(0x6B);
    Wire.write(0x00);
    if (Wire.endTransmission() != 0) return false;

    // 2. Cấu hình Con quay (Gyroscope) dải đo ±500 dps (0x08 -> 65.5 LSB/(deg/s))
    Wire.beginTransmission(addr);
    Wire.write(0x1B);
    Wire.write(0x08);
    Wire.endTransmission();

    // 3. Cấu hình Gia tốc (Accelerometer) dải đo ±8g (0x10 -> 4096 LSB/g)
    Wire.beginTransmission(addr);
    Wire.write(0x1C);
    Wire.write(0x10);
    Wire.endTransmission();

    mpuAddr = addr;
    return true;
}

bool initQMC() {
    if (qmc.begin(0x2C) || qmc.begin(0x0D)) {
        qmc.setMode(QMC5883P_MODE_CONTINUOUS);
        qmc.setRange(QMC5883P_RANGE_2G);
        qmc.setODR(QMC5883P_ODR_50HZ);
        qmc.setOSR(QMC5883P_OSR_8);
        qmc.setDSR(QMC5883P_DSR_1);
        qmc.setSetResetMode(QMC5883P_SETRESET_ON);
        return true;
    }
    return false;
}

void setup() {
    cdh.begin();

    // Kích hoạt hạ áp nuôi cảm biến qua thư viện EPS
    eps.setBuckState(true);
    delay(50); // Đợi mạch nguồn Buck ổn định

    Wire.begin(21, 22);
    Wire.setTimeOut(25);

    // Tự động dò và khởi tạo MPU6500 (0x68 hoặc 0x69) và QMC5883P (0x2C hoặc 0x0D)
    mpuReady = initMPU(0x68) || initMPU(0x69);
    qmcReady = initQMC();
}

void loop() {
    // Tự động nhận diện lại nếu cảm biến khởi động muộn
    if (!mpuReady) {
        mpuReady = initMPU(0x68) || initMPU(0x69);
    }
    if (!qmcReady) {
        qmcReady = initQMC();
    }

    // 1. Đọc dữ liệu Gia tốc & Con quay từ MPU6500 (14 bytes từ 0x3B)
    float ax = 0, ay = 0, az = 0;
    float gx = 0, gy = 0, gz = 0;

    if (mpuReady) {
        Wire.beginTransmission(mpuAddr);
        Wire.write(0x3B);
        if (Wire.endTransmission(false) == 0 && Wire.requestFrom((uint8_t)mpuAddr, (uint8_t)14) == 14) {
            int16_t raw_ax = (Wire.read() << 8) | Wire.read();
            int16_t raw_ay = (Wire.read() << 8) | Wire.read();
            int16_t raw_az = (Wire.read() << 8) | Wire.read();
            Wire.read(); Wire.read(); // Bỏ qua nhiệt độ nội bộ MPU
            int16_t raw_gx = (Wire.read() << 8) | Wire.read();
            int16_t raw_gy = (Wire.read() << 8) | Wire.read();
            int16_t raw_gz = (Wire.read() << 8) | Wire.read();

            // Dải ±8g: 4096 LSB/g, trọng trường g = 9.80665 m/s^2 (đồng bộ với PTITCube library)
            ax = (float)raw_ax * (9.80665f / 4096.0f);
            ay = (float)raw_ay * (9.80665f / 4096.0f);
            az = (float)raw_az * (9.80665f / 4096.0f);

            // Dải ±500 dps: 65.5 LSB/(deg/s), chuyển đổi sang rad/s (đồng bộ với PTITCube library)
            gx = (float)raw_gx * 0.00026646248f;
            gy = (float)raw_gy * 0.00026646248f;
            gz = (float)raw_gz * 0.00026646248f;
        } else {
            mpuReady = false;
        }
    }

    // 2. Tính góc nghiêng (Roll & Pitch)
    float roll  = atan2(ay, az) * 180.0f / (float)PI;
    float pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0f / (float)PI;

    // 3. Đọc dữ liệu từ trường La bàn QMC5883P (uT)
    float mx = 0, my = 0, mz = 0;
    if (qmcReady) {
        float qx, qy, qz;
        if (qmc.getGaussField(&qx, &qy, &qz)) {
            // 1 Gauss = 100 uT
            mx = qx * 100.0f;
            my = qy * 100.0f;
            mz = qz * 100.0f;
        } else {
            qmcReady = false;
        }
    }

    // 4. Tính góc hướng La bàn (Heading: 0 - 360°)
    float heading = atan2(my, mx) * 180.0f / (float)PI;
    if (heading < 0) heading += 360.0f;

    // 5. In 4 dòng viễn trắc chi tiết (tiếng Anh): Gyro, Accel, Mag, Euler
    cdh.printf("Gyro:  X: %6.2f, Y: %6.2f, Z: %6.2f rad/s\n", gx, gy, gz);
    cdh.printf("Accel: X: %6.2f, Y: %6.2f, Z: %6.2f m/s2\n", ax, ay, az);
    cdh.printf("Mag:   X: %6.2f, Y: %6.2f, Z: %6.2f uT\n", mx, my, mz);
    cdh.printf("Roll: %5.1f, Pitch: %5.1f, Head: %5.1f\n", roll, pitch, heading);

    delay(2000);
}

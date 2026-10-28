/**
 * VÍ DỤ 4-2a: ĐỌC CẢM BIẾN QUÁN TÍNH 9 TRỤC (IMU) BẰNG I2C THUẦN
 * 
 * Mô tả:
 * - Đọc trực tiếp thanh ghi MPU6050/MPU6500 qua I2C để lấy Gia tốc & Con quay hồi chuyển.
 * - Tính toán Góc nghiêng vệ tinh (Tilt Angle: Roll & Pitch).
 * - Đọc cảm biến Từ trường QMC5883 và tính góc hướng La bàn (Heading).
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_QMC5883P.h>
#include <math.h>

#define MPU_ADDR 0x68 // Địa chỉ I2C của MPU6050/MPU6500

Adafruit_QMC5883P qmc;
bool qmcReady = false;

void setup() {
    Serial.begin(115200);
    while (!Serial) { delay(10); }

    Serial.println("\n[EXAMPLE] Bắt đầu bài test Cảm biến Quán tính 9 trục (IMU)...");
    Serial.println("[Sensor] Initializing Sensors...");

    // I2C mặc định trên ESP32: SDA=21, SCL=22
    Wire.begin(21, 22);

    // 1. Kiểm tra mã định danh WHO_AM_I của MPU
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x75);
    Wire.endTransmission(false);
    Wire.requestFrom((uint16_t)MPU_ADDR, (uint8_t)1);
    uint8_t whoami = Wire.read();

    if (whoami == 0x68) {
        Serial.println("MPU6050 found!");
    } else if (whoami == 0x70) {
        Serial.println("MPU6500 found!");
    } else if (whoami == 0x71) {
        Serial.println("MPU9250 found!");
    } else {
        Serial.println("MPU fail");
    }

    // Đánh thức MPU khỏi chế độ Sleep (Ghi 0 vào thanh ghi PWR_MGMT_1 - 0x6B)
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x6B);
    Wire.write(0x00);
    Wire.endTransmission();

    // 2. Khởi tạo cảm biến từ trường QMC5883P (0x2C hoặc 0x0D)
    if (qmc.begin(0x2C) || qmc.begin(0x0D)) {
        qmcReady = true;
        Serial.println("QMC5883P found!");
        qmc.setMode(QMC5883P_MODE_CONTINUOUS);
        qmc.setRange(QMC5883P_RANGE_2G);
        qmc.setODR(QMC5883P_ODR_50HZ);
        qmc.setOSR(QMC5883P_OSR_8);
        qmc.setDSR(QMC5883P_DSR_1);
        qmc.setSetResetMode(QMC5883P_SETRESET_ON);
    } else {
        Serial.println("QMC5883P fail");
    }
}

void loop() {
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 1000) {
        lastPrint = millis();

        // Đọc dữ liệu Gia tốc và Vận tốc góc từ thanh ghi MPU (0x3B, 14 bytes)
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
            Wire.read(); Wire.read(); // Bỏ qua 2 bytes nhiệt độ
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

        // Tính Góc nghiêng (Roll & Pitch)
        float roll = atan2(ay, az) * 180.0 / PI;
        float pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;

        // Đọc Từ trường (uT) và tính Heading
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
        float heading = atan2(my, mx) * 180.0 / PI;
        if (heading < 0) heading += 360.0;

        Serial.println("==================================================");

        // 1. Dữ liệu Gia tốc (m/s^2)
        Serial.println(">>> GIA TỐC (ACCELEROMETER) <<<");
        Serial.printf("  X: %6.2f m/s^2 | Y: %6.2f m/s^2 | Z: %6.2f m/s^2\n", 
                      ax, ay, az);

        // 2. Dữ liệu Vận tốc góc (rad/s)
        Serial.println(">>> CON QUAY HỒI CHUYỂN (GYROSCOPE) <<<");
        Serial.printf("  X: %6.2f rad/s | Y: %6.2f rad/s | Z: %6.2f rad/s\n", 
                      gx, gy, gz);

        // 3. Góc nghiêng (Tilt Angle)
        Serial.println(">>> GÓC NGHIÊNG (TILT ANGLE) <<<");
        Serial.printf("  Roll (Lắc ngang)  : %6.2f° | Pitch (Chúc/Ngóc) : %6.2f°\n", 
                      roll, pitch);

        // 4. Dữ liệu Từ trường (uT) & La bàn (Heading)
        Serial.println(">>> LA BÀN TỪ TRƯỜNG (MAGNETOMETER) <<<");
        Serial.printf("  X: %6.2f uT    | Y: %6.2f uT    | Z: %6.2f uT    | Heading: %6.2f°\n", 
                      mx, my, mz, heading);

        Serial.println("==================================================\n");
    }
}


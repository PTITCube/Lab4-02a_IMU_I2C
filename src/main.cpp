/**
 * VÍ DỤ 4-2a: ĐỌC CẢM BIẾN IMU (MPU6050/6500) & TỪ TRƯỜNG (QMC5883P) QUA I2C
 * 
 * Mô tả:
 * - Đọc trực tiếp thanh ghi thô của MPU6050/MPU6500 qua I2C để lấy dữ liệu Gia tốc & Con quay hồi chuyển.
 * - Tính toán Góc nghiêng vệ tinh (Tilt Angle: Roll & Pitch).
 * - Đọc dữ liệu cảm biến Từ trường (Magnetometer) và tính hướng La bàn (Heading).
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

    Serial.println("\n[RAW] Khởi tạo giao tiếp I2C cho IMU & Cảm biến Từ trường...");
    
    // I2C mặc định trên ESP32: SDA=21, SCL=22
    Wire.begin(21, 22); 
    
    // 1. Đánh thức MPU6050/MPU6500 từ chế độ Sleep (Ghi 0 vào thanh ghi PWR_MGMT_1 - 0x6B)
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x6B); // Thanh ghi Power Management 1
    Wire.write(0x00); // 0 = Wake up
    if (Wire.endTransmission(true) == 0) {
        Serial.println("[RAW] MPU6050/6500 đã được đánh thức thành công.");
    } else {
        Serial.println("[ERROR] Không thể kết nối tới MPU ở địa chỉ 0x68!");
    }

    // 2. Khởi tạo cảm biến Từ trường QMC5883P (Địa chỉ mặc định 0x2C hoặc 0x0D)
    if (qmc.begin(0x2C)) {
        qmcReady = true;
        Serial.println("[RAW] Cảm biến từ trường QMC5883P đã sẵn sàng (0x2C).");
    } else if (qmc.begin(0x0D)) {
        qmcReady = true;
        Serial.println("[RAW] Cảm biến từ trường QMC5883L đã sẵn sàng (0x0D).");
    } else {
        Serial.println("[WARN] Không tìm thấy cảm biến từ trường QMC5883 qua thư viện, thử raw I2C...");
    }

    if (qmcReady) {
        qmc.setMode(QMC5883P_MODE_CONTINUOUS);
        qmc.setRange(QMC5883P_RANGE_2G);
        qmc.setODR(QMC5883P_ODR_50HZ);
    }
}

void loop() {
    // ------------------------------------------------------------------------
    // 1. ĐỌC DỮ LIỆU GIA TỐC VÀ TÍNH GÓC NGHIÊNG (ROLL & PITCH) TỪ MPU
    // ------------------------------------------------------------------------
    // Đọc từ thanh ghi 0x3B: 6 bytes Gia tốc (ACCEL_XOUT_H -> ACCEL_ZOUT_L)
    //                      2 bytes Nhiệt độ (TEMP_OUT_H -> TEMP_OUT_L)
    //                      6 bytes Con quay hồi chuyển (GYRO_XOUT_H -> GYRO_ZOUT_L)
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    
    Wire.requestFrom((uint16_t)MPU_ADDR, (uint8_t)14, (bool)true);
    
    if (Wire.available() >= 14) {
        int16_t raw_ax = (Wire.read() << 8) | Wire.read();
        int16_t raw_ay = (Wire.read() << 8) | Wire.read();
        int16_t raw_az = (Wire.read() << 8) | Wire.read();
        int16_t raw_temp = (Wire.read() << 8) | Wire.read();
        int16_t raw_gx = (Wire.read() << 8) | Wire.read();
        int16_t raw_gy = (Wire.read() << 8) | Wire.read();
        int16_t raw_gz = (Wire.read() << 8) | Wire.read();

        // MPU6050 mặc định dải đo ±2g (độ nhạy 16384 LSB/g)
        float ax_g = (float)raw_ax / 16384.0;
        float ay_g = (float)raw_ay / 16384.0;
        float az_g = (float)raw_az / 16384.0;

        // Tính Góc nghiêng (Tilt Angle): Roll và Pitch theo công thức lượng giác (đơn vị: Độ - Degree)
        // Roll: góc lắc nghiêng quanh trục X
        // Pitch: góc chúc/ngóc quanh trục Y
        float roll = atan2(ay_g, az_g) * 180.0 / PI;
        float pitch = atan2(-ax_g, sqrt(ay_g * ay_g + az_g * az_g)) * 180.0 / PI;

        // Con quay hồi chuyển (dải đo mặc định ±250 deg/s: 131 LSB/(deg/s))
        float gx_dps = (float)raw_gx / 131.0;
        float gy_dps = (float)raw_gy / 131.0;
        float gz_dps = (float)raw_gz / 131.0;

        Serial.println("==================================================");
        Serial.println(">>> 1. GIA TỐC & GÓC NGHIÊNG (ACCELEROMETER & TILT) <<<");
        Serial.printf("  Gia tốc thô (Raw)   : X=%6d | Y=%6d | Z=%6d\n", raw_ax, raw_ay, raw_az);
        Serial.printf("  Gia tốc (g)         : X=%6.2f | Y=%6.2f | Z=%6.2f g\n", ax_g, ay_g, az_g);
        Serial.printf("  => Góc nghiêng Roll (Lắc ngang)  : %6.2f°\n", roll);
        Serial.printf("  => Góc nghiêng Pitch (Chúc/Ngóc) : %6.2f°\n", pitch);
        Serial.printf("  Tốc độ góc Gyro     : X=%6.2f | Y=%6.2f | Z=%6.2f dps\n", gx_dps, gy_dps, gz_dps);
    } else {
        Serial.println("[ERROR] Không đọc được dữ liệu từ MPU6050/6500 qua I2C.");
    }

    // ------------------------------------------------------------------------
    // 2. ĐỌC DỮ LIỆU TỪ TRƯỜNG & TÍNH GÓC HƯỚNG LA BÀN (MAGNETOMETER & HEADING)
    // ------------------------------------------------------------------------
    if (qmcReady) {
        float mx = 0, my = 0, mz = 0;
        if (qmc.getGaussField(&mx, &my, &mz)) {
            // Chuyển đổi từ Gauss sang microTesla (1 Gauss = 100 uT)
            float mx_uT = mx * 100.0;
            float my_uT = my * 100.0;
            float mz_uT = mz * 100.0;

            // Tính góc phương vị la bàn (Yaw / Heading) từ trục từ trường X, Y
            float heading = atan2(my, mx) * 180.0 / PI;
            if (heading < 0) heading += 360.0;

            Serial.println(">>> 2. TỪ TRƯỜNG & HƯỚNG LA BÀN (MAGNETOMETER) <<<");
            Serial.printf("  Từ trường (Gauss)   : X=%6.3f | Y=%6.3f | Z=%6.3f G\n", mx, my, mz);
            Serial.printf("  Từ trường (uT)      : X=%6.2f | Y=%6.2f | Z=%6.2f uT\n", mx_uT, my_uT, mz_uT);
            Serial.printf("  => Góc hướng Heading (La bàn)    : %6.2f°\n", heading);
        } else {
            Serial.println("[ERROR] Lỗi khi đọc dữ liệu từ trường từ QMC5883!");
        }
    } else {
        // Dự phòng đọc trực tiếp thanh ghi I2C nếu thư viện chưa nhận
        Wire.beginTransmission(0x0D);
        Wire.write(0x00);
        if (Wire.endTransmission() == 0) {
            Wire.requestFrom((uint16_t)0x0D, (uint8_t)6);
            if (Wire.available() >= 6) {
                int16_t qx = Wire.read() | (Wire.read() << 8);
                int16_t qy = Wire.read() | (Wire.read() << 8);
                int16_t qz = Wire.read() | (Wire.read() << 8);
                float heading = atan2((float)qy, (float)qx) * 180.0 / PI;
                if (heading < 0) heading += 360.0;
                Serial.println(">>> 2. TỪ TRƯỜNG (RAW I2C QMC5883L 0x0D) <<<");
                Serial.printf("  Từ trường thô: X=%d | Y=%d | Z=%d | Heading=%.2f°\n", qx, qy, qz, heading);
            }
        }
    }

    Serial.println("==================================================\n");
    delay(1000);
}

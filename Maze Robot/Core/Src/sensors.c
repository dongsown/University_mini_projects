#include "sensors.h"
#include "i2c.h"      
#include "VL53L0X.h"
#include <stdbool.h>

// --- CÁC ĐỊNH NGHĨA ---
#define TCA9548A_ADDR 0x70
#define VL53L0X_ADDR  0x29
#define SENSOR_REINIT_ATTEMPTS 5 // Số lần thử khởi tạo lại cảm biến

// --- CÁC HẰNG SỐ HIỆU CHỈNH ---
const float SENSOR_SCALE = 0.99f;
const int SENSOR_OFFSET = 18;

// --- BIẾN TOÀN CỤC CỦA MODULE ---
uint16_t sensor_distances[NUM_SENSORS];
static uint8_t reinit_count = 0; // Biến đếm số lần khởi tạo lại

// --- KHAI BÁO HÀM NỘI BỘ (STATIC) ---
static void select_sensor_channel(uint8_t channel);
static bool init_single_sensor(void);

/**
 * @brief Chọn kênh I2C trên bộ ghép kênh TCA9548A.
 * @param channel: Kênh cần chọn (0-7)
 */
static void select_sensor_channel(uint8_t channel) {
    if (channel >= NUM_SENSORS) return;
    uint8_t data = 1 << channel;
    HAL_I2C_Master_Transmit(&hi2c1, (TCA9548A_ADDR << 1), &data, 1, 100);
}

/**
 * @brief Khởi tạo một cảm biến VL53L0X duy nhất trên kênh I2C hiện tại.
 * @return true nếu khởi tạo thành công, false nếu thất bại.
 */
static bool init_single_sensor(void) {
    if (initVL53L0X(1, &hi2c1)) {
        setSignalRateLimit(2000);
        setMeasurementTimingBudget(22 * 1000UL); 
        startContinuous(0); 
        return true;
    }
    return false;
}

/**
 * @brief Khởi tạo tất cả các cảm biến khoảng cách.
 */
void sensors_init(void) {
    setTimeout(100);

    for (int i = 0; i < NUM_SENSORS; i++) {
        select_sensor_channel(i);
        HAL_Delay(10); 
        // Kiểm tra xem có thiết bị ở địa chỉ VL53L0X không
        if (HAL_I2C_IsDeviceReady(&hi2c1, (VL53L0X_ADDR << 1), 2, 50) == HAL_OK) {
            init_single_sensor();
        } else {
            // Có thể thêm log hoặc xử lý lỗi ở đây nếu muốn
            sensor_distances[i] = 65535; // Gán giá trị lỗi mặc định
        }
    }
}

/**
 * @brief Đọc giá trị từ tất cả cảm biến với cơ chế xử lý lỗi mạnh mẽ.
 */
void sensors_read_all(void) {
    statInfo_t_VL53L0X distanceStr;

    for (int i = 0; i < NUM_SENSORS; i++) {
        select_sensor_channel(i);
        
        // --- LOGIC XỬ LÝ LỖI MẠNH MẼ ĐƯỢC TÍCH HỢP ---
        if (HAL_I2C_IsDeviceReady(&hi2c1, (VL53L0X_ADDR << 1), 2, 10) != HAL_OK) {
            sensor_distances[i] = 65535;
            continue; // Chuyển sang cảm biến tiếp theo
        }

        uint16_t raw_distance = readRangeContinuousMillimeters(&distanceStr);

        // Nếu cảm biến bị lỗi và trả về 65535
        if (raw_distance == 65535) {
            // Cố gắng khởi tạo lại cảm biến
            while (!init_single_sensor()) {
                reinit_count++;
                HAL_Delay(200); // Chờ một chút trước khi thử lại
                
                // Nếu thử quá nhiều lần mà không được -> Reset hệ thống
                if (reinit_count > SENSOR_REINIT_ATTEMPTS) {
                    HAL_Delay(1000); // Chờ 1 giây để đảm bảo mọi thứ ổn định
                    NVIC_SystemReset(); // Lệnh reset cứng
                    break; // Mặc dù sẽ không chạy đến đây sau khi reset
                }
            }

            // Nếu khởi tạo lại thành công
            if (reinit_count <= SENSOR_REINIT_ATTEMPTS) {
                reinit_count = 0; // Reset bộ đếm
                HAL_Delay(50);    // Chờ cảm biến ổn định
                raw_distance = readRangeContinuousMillimeters(&distanceStr); // Đọc lại giá trị
            }
        }

        // Hiệu chỉnh và lưu giá trị
        if (raw_distance < 8190) { 
            sensor_distances[i] = (uint16_t)(raw_distance * SENSOR_SCALE - SENSOR_OFFSET);
        } else {
            sensor_distances[i] = raw_distance; 
        }
    }
}


// --- CÁC HÀM GETTER ĐỂ TRUY CẬP DỮ LIỆU ---

uint16_t get_left_distance(void) {
    return sensor_distances[1];
}

uint16_t get_right_distance(void) {
    return sensor_distances[2];
}

uint16_t get_front_distance(void) {
    return sensor_distances[0];
}


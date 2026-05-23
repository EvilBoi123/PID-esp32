#include <stdio.h>
#include <math.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_err.h>
#include <esp_log.h>
#include <mpu6050.h>

#define I2C_SDA  21
#define I2C_SCL  27
#define ADDR     MPU6050_I2C_ADDRESS_LOW

// Complementary filter coefficient
// Higher = trust gyro more (smoother but drifts)
// Lower  = trust accel more (noisy but no drift)
#define ALPHA       0.98f
#define LOOP_MS     10          // 10ms = 100Hz
#define DT          (LOOP_MS / 1000.0f)

static const char *TAG = "imu";

void mpu6050_task(void *pvParameters)
{
    mpu6050_dev_t dev = { 0 };
    ESP_ERROR_CHECK(mpu6050_init_desc(&dev, ADDR, 0, I2C_SDA, I2C_SCL));

    while (1) {
        if (i2c_dev_probe(&dev.i2c_dev, I2C_DEV_WRITE) == ESP_OK) {
            ESP_LOGI(TAG, "MPU6050 found");
            break;
        }
        ESP_LOGE(TAG, "MPU6050 not found, retrying...");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    ESP_ERROR_CHECK(mpu6050_init(&dev));

    float roll  = 0.0f;
    float pitch = 0.0f;
    bool first  = true;

    while (1) {
        mpu6050_acceleration_t accel = { 0 };
        mpu6050_rotation_t gyro      = { 0 };

        ESP_ERROR_CHECK(mpu6050_get_motion(&dev, &accel, &gyro));

        // --- Accel-derived angle (absolute but noisy) ---
        float accel_roll  = atan2f(accel.y, accel.z) * 180.0f / M_PI;
        float accel_pitch = atan2f(-accel.x,
                             sqrtf(accel.y * accel.y + accel.z * accel.z))
                            * 180.0f / M_PI;

        if (first) {
            // Seed with accel on first iteration to avoid startup jump
            roll  = accel_roll;
            pitch = accel_pitch;
            first = false;
        } else {
            // --- Gyro integration (relative but smooth) ---
            float gyro_roll  = roll  + gyro.x * DT;
            float gyro_pitch = pitch + gyro.y * DT;

            // --- Complementary filter: blend both ---
            roll  = ALPHA * gyro_roll  + (1.0f - ALPHA) * accel_roll;
            pitch = ALPHA * gyro_pitch + (1.0f - ALPHA) * accel_pitch;
        }

        // In-place terminal update
        if (!first) printf("\033[2A");
        printf("Roll:  %8.2f deg\033[K\n", roll);
        printf("Pitch: %8.2f deg\033[K\n", pitch);

        vTaskDelay(pdMS_TO_TICKS(LOOP_MS));
    }
}

void app_main(void)
{
    ESP_ERROR_CHECK(i2cdev_init());
    xTaskCreate(mpu6050_task, "mpu6050_task", configMINIMAL_STACK_SIZE * 6, NULL, 5, NULL);
}

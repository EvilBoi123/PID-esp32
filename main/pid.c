#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdio.h>
#include <math.h>


#include "ledc.h"
#include "pid.h"
#include "mpu6050.h"

#define LOOP_MS     10          // 10ms = 100Hz
#define DT          (LOOP_MS / 1000.0f)


// From mpu6050 file
extern mpu6050_dev_t dev; 
extern float roll; 
extern float pitch;
extern void enable_mpu(void);

#define BASE_SETPOINT  30.0f
static float speed_top_left    = 0.0f;
static float speed_bottom_left = 0.0f;

Fan_config_t top_left = { "Top left", BASE_SETPOINT, -1, LEDC_CHANNEL_1, &speed_top_left }; 
Fan_config_t bottom_left = { "Bottom left", BASE_SETPOINT, 1, LEDC_CHANNEL_2, &speed_bottom_left };

void pid_init(PIDController *pid, float kp, float ki, float kd, float min_out, float max_out){
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral = 0;
    pid->prev_error = 0;
    pid->output_min = min_out;
    pid->output_max = max_out;
}

float pid_compute (PIDController *pid, float setpoint, float input, float dt){
	float error = setpoint - input; 
        //pid->integral += error * dt; 
	
	float integral_contribution = pid->ki * (pid->integral + error * dt);
    if (integral_contribution < pid->output_max &&
        integral_contribution > pid->output_min) {
        pid->integral += error * dt;
    }	


        float derivative = (error - pid->prev_error) / dt; 

	float output = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative; 

	if (output > pid->output_max) output = pid->output_max;
    if (output < pid->output_min) output = pid->output_min;

    pid->prev_error = error;
    return output;	
}

void pid_task(void *pvParameters){
	Fan_config_t *config = (Fan_config_t *)pvParameters;
	float speed; 
	PIDController pid_handler; 
	pid_init(&pid_handler, 0.05313, 0.005f, 0.000495f, 0.0, 255.0f); 
	while (1){
		float effective_roll = roll * config->direction;
		speed = pid_compute(&pid_handler, config->target_setpoint, effective_roll, DT); 
		if (config->direction > 0)
            	 speed_bottom_left = speed;
        	else
            	 speed_top_left = speed;

		*config->value = speed;
		vTaskDelay(pdMS_TO_TICKS(LOOP_MS));
	}; 
}

void print_task(void *pvParameters)
{
    bool first = true;
    while (1) {
        if (!first) printf("\033[4A");   // move up 4 lines

        printf("[Top left]    Target: %5.1f | Roll: %6.2f deg | Speed: %6.2f\033[K\n",
               top_left.target_setpoint, roll, speed_top_left);
        printf("[Bottom left] Target: %5.1f | Roll: %6.2f deg | Speed: %6.2f\033[K\n",
               bottom_left.target_setpoint, roll, speed_bottom_left);
        printf("[Roll]  %8.2f deg\033[K\n", roll);
        printf("[Pitch] %8.2f deg\033[K\n", pitch);
	
	//printf("[I_top] %8.3f  [I_bot] %8.3f\033[K\n", speed_top_left, speed_bottom_left);
        first = false;
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void app_main(){
    
    //ESP_ERROR_CHECK(i2cdev_init());
    //xTaskCreate(mpu6050_task, "mpu6050_task", configMINIMAL_STACK_SIZE * 6, NULL, 5, NULL);
    enable_mpu();
    xTaskCreate(pid_task, "bottom_left", 4096, (void *)&bottom_left, 5, NULL);
    xTaskCreate(pid_task, "top_left", 4096, (void *)&top_left, 5, NULL);
    xTaskCreate(print_task, "print",      4096, NULL, 4, NULL);

    motor_init();
    xTaskCreate(motor_throttle, "motor_bottom_left", 4096, (void *)&bottom_left, 5, NULL);
    xTaskCreate(motor_throttle, "motor_top_left", 4096, (void *)&top_left, 5, NULL);

};




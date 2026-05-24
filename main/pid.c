#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdio.h>
#include <math.h>

#include "pid.h"
#include "mpu6050.h"

#define LOOP_MS     10          // 10ms = 100Hz
#define DT          (LOOP_MS / 1000.0f)

extern mpu6050_dev_t dev; 
extern float roll; 
extern float pitch;
extern void enable_mpu(void);

#define BASE_SETPOINT  30.0f
static float speed_top_left    = 0.0f;
static float speed_bottom_left = 0.0f;

Fan_config_t top_left = { "Top left", BASE_SETPOINT, -1 }; 
Fan_config_t bottom_left = { "Bottom left", BASE_SETPOINT, 1 };

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
	//float error = pid->setpoint - input; 
	float error = setpoint - input; 
        pid->integral += error * dt; 
        float derivative = (error - pid->prev_error) / dt; 
	

	float output = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative; 

	if (output > pid->output_max) output = pid->output_max;
    if (output < pid->output_min) output = pid->output_min;

    pid->prev_error = error;
    return output;	
}

void pid_task(void *pvParameters){
	Fan_config_t *config = (Fan_config_t *)pvParameters;
	//float fan_speed; 
	float speed; 
	//bool first = true; 
	PIDController pid_handler; 
	pid_init(&pid_handler, 1.0, 0.06, 0.02, 0.0, 100.0); 
	while (1){
		float effective_roll = roll * config->direction;
		speed = pid_compute(&pid_handler, config->target_setpoint, effective_roll, DT); 
		//if (!first) printf("\033[3A");
        	//printf("Current Roll:  %8.2f deg\033[K\n", roll);
        	//printf("Current Pitch: %8.2f deg\033[K\n", pitch);
        	//printf("PID Fan Speed: %8.2f Output\033[K\n", fan_speed);
		//printf("\n[%s] Target: %5.1f Roll: %6.2f deg\033[K Output Speed: %6.2f\033[K", config->name, config->target_setpoint, roll, fan_speed);
		//printf("[%s] Target: %5.1f | Roll: %6.2f deg | Output Speed: %6.2f\n", config->name, config->target_setpoint, roll, fan_speed);
        	//first = false;
		
		if (config->direction > 0)
            	 speed_bottom_left = speed;
        	else
            	 speed_top_left = speed;
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

};




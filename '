#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdio.h>
#include <math.h>

#include "mpu6050.c"
#include "pid.h"

#define LOOP_MS     10          // 10ms = 100Hz
#define DT          (LOOP_MS / 1000.0f)

extern mpu6050_dev_t dev; 
extern float roll; 
extern float pitch;
extern void enable_mpu(void);

void pid_init(PIDController *pid, float kp, float ki, float kd, float min_out, float max_out){
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral = 0;
    pid->prev_error = 0;
    pid->output_min = min_out;
    pid->output_max = max_out;
}

float pid_compute (PIDController *pid, float input, float dt){
	//float error = pid->setpoint - input; 
	float error = 30 - input; 
        pid->integral += error * dt; 
        float derivative = (error - pid->prev_error) / dt; 
	

	float output = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative; 

	if (output > pid->output_max) output = pid->output_max;
    if (output < pid->output_min) output = pid->output_min;

    pid->prev_error = error;
    return output;	
}

void pid_task(void *pvParameters){
	float fan_speed; 
	bool first = true; 
	PIDController pid_handler; 
	pid_init(&pid_handler, 1.0, 0.9, 0.02, 0.0, 100.0); 
	while (1){
		fan_speed = pid_compute(&pid_handler, roll, DT); 
		if (!first) printf("\033[3A");
        	printf("Current Roll:  %8.2f deg\033[K\n", roll);
        	printf("Current Pitch: %8.2f deg\033[K\n", pitch);
        	printf("PID Fan Speed: %8.2f Output\033[K\n", fan_speed);
        	first = false;
		vTaskDelay(pdMS_TO_TICKS(LOOP_MS));
	}; 
}

void app_main(){
    
    //ESP_ERROR_CHECK(i2cdev_init());
    //xTaskCreate(mpu6050_task, "mpu6050_task", configMINIMAL_STACK_SIZE * 6, NULL, 5, NULL);
    enable_mpu();
    xTaskCreate(pid_task, "pid_task", 4096, NULL, 5, NULL);

};




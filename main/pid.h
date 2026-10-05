#include <freertos/FreeRTOS.h>
#include <stdio.h> 
#include "driver/ledc.h"

typedef struct {
    float kp;
    float ki;
    float kd;

    float setpoint;
    float integral;
    float prev_error;

    float output_min;
    float output_max;
} PIDController;

typedef struct {
	const char *name; 
	float target_setpoint; 
	int direction;
	ledc_channel_t channel;
	float *value; 

} Fan_config_t;

void pid_init(PIDController* pid, float kp, float ki, float kd, float min_out, float max_out);
float pid_compute(PIDController* pid, float setpoint, float input, float dt);

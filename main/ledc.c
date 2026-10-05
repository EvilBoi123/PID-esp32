#include <freertos/FreeRTOS.h>
#include <stdio.h> 
#include "driver/ledc.h"

#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_RESOLUTION LEDC_TIMER_8_BIT
#define LEDC_FREQUENCY (5000)

#define MOTOR_1_GPIO    18
#define MOTOR_2_GPIO    19
#define MOTOR_3_GPIO    21
#define MOTOR_4_GPIO    22

#include "pid.h"

typedef struct {
	ledc_channel_config_t channel; 
	float value; 
}which_fan_t; 

void motor_init(){
	ledc_timer_config_t ledc_timer = {
		.speed_mode = LEDC_MODE, 
		.timer_num = LEDC_TIMER, 
		.duty_resolution = LEDC_TIMER_8_BIT, 
		.freq_hz = 20000, 
		.clk_cfg = LEDC_AUTO_CLK
	}; 
	ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

	ledc_channel_config_t ch_motor1 = {
		.channel = LEDC_CHANNEL_1, 
		.speed_mode = LEDC_MODE, 
		.gpio_num = MOTOR_1_GPIO, 
		.intr_type = LEDC_INTR_DISABLE, 
		.duty = 0, 
		.hpoint = 0
	}; 

	ledc_channel_config_t ch_motor2 = ch_motor1; 
	ch_motor2.gpio_num = MOTOR_2_GPIO; 
	ch_motor2.channel = LEDC_CHANNEL_2; 

	ESP_ERROR_CHECK(ledc_channel_config(&ch_motor1));
	ESP_ERROR_CHECK(ledc_channel_config(&ch_motor2));

}

void motor_throttle(void *pvParameters){
	//which_fan_t *fan = (which_fan_t *)pvParameters 
	Fan_config_t *fan = (Fan_config_t *)pvParameters; 

	while(1){
	ledc_set_duty(LEDC_LOW_SPEED_MODE, fan->channel, (uint32_t)*fan->value); 
	ledc_update_duty(LEDC_LOW_SPEED_MODE, fan->channel);
	}
}

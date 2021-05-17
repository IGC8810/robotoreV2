#ifndef peripheral_func_H
#define peripheral_func_H

#include "main.h"
#include "AQM0802.h"
#include "ICM20648.h"
#include "INA260.h"
#include "Flash_F405.h"
#include "calculate.h"
#include "control.h"
#include "math.h"
/*
#include "arm_math.h"
*/

extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;
extern TIM_HandleTypeDef htim8;
extern TIM_HandleTypeDef htim12;
extern UART_HandleTypeDef huart1;
extern uint16_t line_sen0;
extern uint16_t line_sen1;
extern uint16_t line_sen2;
extern uint16_t line_sen3;
extern uint16_t line_sen4;
extern uint16_t line_sen5;
extern uint16_t line_sen6;
extern uint16_t line_sen7;
extern uint16_t line_sen8;
extern uint16_t line_sen9;
extern uint16_t line_sen10;
extern uint16_t line_sen11;
extern int64_t enc_tim1_total;
extern int64_t enc_tim8_total;
extern int8_t setup_mode;
extern int8_t check_sens_val;
extern uint8_t sw_center_state;

#define ADC_DATA_BUFFR_SIZE		((uint16_t)12)

#define LED_R_SET 		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET)
#define LED_G_SET 		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET)
#define LED_B_SET 		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET)
#define LED_R_RESET 	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET)
#define LED_G_RESET 	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET)
#define LED_B_RESET 	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET)

#define MR_SET 			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET)
#define MR_RESET 		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET)
#define ML_SET 			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET)
#define ML_RESET 		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET)

#define CHECK_SENS_MAX	11

#define COUNTER_PERIOD_TIM12	839

#define ESC_MAX 		3527	//84[us]
#define ESC_MIN			1763	//42[us]

#define ENC_PULSE_MM 	0.012207f		//0.024414f 2逓倍
// 速度 = 1msでカウントしたパルス　* 1パルスで進む距離 * 1000 [mm/s]
// 1msで数mm進むのでm/s これに1000かけてmm/s
// 1pulseで進む距離　= タイヤ周長 / (エンコーダのパルス * n逓倍 * 減速比)
//								= 68mm / (512 * 4 * 2.72) = 0.0122...

void peripheral_init(void);
void gpio_set(void);
void led_pattern(uint8_t);
void getEncoder(void);
void ADval_get(void);
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef*);
void MotorCtrl(int16_t, int16_t, uint8_t);
void buzzer(uint8_t);
void HAL_GPIO_EXTI_Callback(uint16_t);

#endif

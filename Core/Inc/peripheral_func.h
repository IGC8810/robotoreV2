#ifndef peripheral_func_H
#define peripheral_func_H

#include "main.h"
#include "AQM0802.h"
#include "ICM20648.h"
#include "INA260.h"
#include "Flash_F405.h"
#include "calculate.h"
#include "control.h"
#include "arm_math.h"

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
extern uint16_t line_sen12;
extern uint16_t line_sen13;
extern uint16_t line_senLLL;
extern uint16_t line_senLL;
extern uint16_t line_senL;
extern uint16_t line_senR;
extern uint16_t line_senRR;
extern uint16_t line_senRRR;
extern int64_t enc_tim1_total;
extern int64_t enc_tim8_total;
extern int64_t enc_tim_total;
extern int32_t enc_tim1_cnt_10ms;
extern int32_t enc_tim8_cnt_10ms;
extern int64_t enc_cnt;
extern int64_t enc_cnt2;
extern float velR;
extern float velL;
extern int posR;
extern int posL;
extern char error_flag;
extern uint16_t error_cnt;
extern int timer;
extern float target_vel;
extern unsigned char main_pattern;
extern uint8_t maker_check;
extern char crossline_flag;
extern unsigned char velocity_pattern;
extern int encoder_event;
extern uint8_t flash_flag;
extern uint32_t log_adress;
extern uint32_t* flash_read_test;
extern uint16_t maker_cnt;
extern int16_t log_array;
extern uint32_t plan_velo_adress;
extern uint8_t second_trace_flag;
extern float mm_total;
extern uint32_t maker_adress;
extern float maker_distance_L[];
extern float maker_distance_R[];
extern uint16_t maker_distance_cmp_lim;
extern float log_zg;
extern float log_mm;
extern float PlanVelo[];
extern float PlanVelo2[];
extern uint8_t second_trace_pattern;
extern int8_t setup_mode;
extern int8_t check_sens_val;
extern uint8_t sw_up_state;
extern uint8_t sw_center_state;
extern uint8_t cnt_sw;

#define ADC_DATA_BUFFR_SIZE		((uint16_t)14)

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

#define CHECK_SENS_MAX	14

#define COUNTER_PERIOD_TIM12	839

#define ESC_MAX 		3527	//84[us]
#define ESC_MIN			1763	//42[us]

#define ENC_PULSE_MM 	0.012207f		//0.024414f 2逓倍
// 速度 = 1msでカウントしたパルス　* 1パルスで進む距離 * 1000 [mm/s]
// 1msで数mm進むのでm/s これに1000かけてmm/s
// 1pulseで進む距離　= タイヤ周長 / (エンコーダのパルス * n逓倍 * 減速比)
//								= 68mm / (512 * 4 * 2.72) = 0.0122...

#define MAX_VELOCITY	6000.0f		//[mm/s]5000
#define MIN_VELOCITY	1100.0f		//[mm/s]1000
#define START_VELOCITY	1100.0f		//[mm/s]1000
#define ACCELERATION	10.0f		//[mm/s^2]50
#define DECELERATION	10.0f		//[mm/s^2]10
#define END_VELOCITY	1000.0f		//[mm/s]1000
#define END_DISTANCE    250.0f		//[mm]

#define ERRORCHECK		25000		//25000←9号館
#define CROSSCHECK		2500		//2500←9号館
#define MAKERTHRESHOLD	1600		//900←9号館

#define MAX_VELOCITY2	7000.0f		//[mm/s]7000
#define MIN_VELOCITY2	1300.0f		//[mm/s]1200
#define START_VELOCITY2	1300.0f		//[mm/s]1200
#define ACCELERATION2	20.0f		//[mm/s^2]100
#define DECELERATION2	20.0f		//[mm/s^2]10
#define END_VELOCITY2	1400.0f		//[mm/s]1200
#define END_DISTANCE2   250.0f		//[mm]

void peripheral_init(void);
void gpio_set(void);
void led_pattern(uint8_t);
void getEncoder(void);
void ADval_get(void);
void ADval_sum(void);
uint8_t MakerSenTh(uint16_t);
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef*);
void MotorCtrl(int16_t, int16_t, uint8_t);
void buzzer(uint8_t);
void HAL_GPIO_EXTI_Callback(uint16_t);

#endif

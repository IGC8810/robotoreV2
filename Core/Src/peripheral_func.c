#include "peripheral_func.h"

# ifdef __GNUC__
 #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
# else
 #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
# endif /*__GNUC__*/

PUTCHAR_PROTOTYPE {
 HAL_UART_Transmit(&huart1, (uint8_t*)&ch, 1, 0xFFFF);
 return ch;
}

uint16_t ADC1_Buff[ADC_DATA_BUFFR_SIZE];
uint16_t line_sen0;
uint16_t line_sen1;
uint16_t line_sen2;
uint16_t line_sen3;
uint16_t line_sen4;
uint16_t line_sen5;
uint16_t line_sen6;
uint16_t line_sen7;
uint16_t line_sen8;
uint16_t line_sen9;
uint16_t line_sen10;
uint16_t line_sen11;
int16_t enc_tim1_ms = 0;
int16_t enc_tim8_ms = 0;
int64_t enc_tim1_total = 0;
int64_t enc_tim8_total = 0;
int64_t enc_tim_total = 0;
int8_t setup_mode = 0;
int8_t check_sens_val = 0;
uint8_t sw_center_state = 0;

void peripheral_init(void){
	gpio_set();
	lcd_init();
	INA260_init();
	if( IMU_init() == 1 ) {
		lcd_locate(0,0);
		lcd_print("WHO_AM_I");
		lcd_locate(0,1);
		lcd_print("SUCCESS");
	}
	else {
		lcd_locate(0,0);
		lcd_print("WHO_AM_I");
		lcd_locate(0,1);
		lcd_print("Failed");
	}
	//set_encoder
	HAL_TIM_Encoder_Start(&htim1,TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim8,TIM_CHANNEL_ALL);
	//set_motordrive
	HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);
	__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, 0);
	//set_buzzer
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
	//set_4in1-esc
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, ESC_MIN);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, ESC_MIN);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, ESC_MIN);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, ESC_MIN);
	//set_timer
	HAL_TIM_Base_Start_IT(&htim6);
	HAL_TIM_Base_Start_IT(&htim7);
	//ADC
	HAL_ADC_Start_DMA(&hadc1, (uint32_t *) ADC1_Buff, ADC_DATA_BUFFR_SIZE);

}

void gpio_set(void){
	CS_SET;
	MR_SET;
	ML_SET;
	LED_R_SET;
	LED_G_SET;
	LED_B_SET;
}

void led_pattern(uint8_t led){
	if(led & 0b001) LED_R_RESET;
		else LED_R_SET;
	if(led & 0b010) LED_G_RESET;
		else LED_G_SET;
	if(led & 0b100) LED_B_RESET;
		else LED_B_SET;
}

void getEncoder(void) {

	enc_tim1_ms = TIM1 -> CNT;
	enc_tim8_ms = TIM8 -> CNT;

	TIM1 -> CNT = 0;
	TIM8 -> CNT = 0;

	enc_tim1_total += enc_tim1_ms;
	enc_tim8_total += enc_tim8_ms;
	enc_tim_total = (enc_tim1_total + enc_tim8_total) / 2;

}

//sensor-borad from left to right		AD9 AD8 AD15 AD14 AD7 AD6   AD5 AD4 AD3 AD2 AD1 AD0
//maker-borad  from left to right	 	ADxx 	ADxx
void ADval_get(void) {
	line_sen0  = ADC1_Buff[0];
	line_sen1  = ADC1_Buff[1];
	line_sen2  = ADC1_Buff[2];
	line_sen3  = ADC1_Buff[3];
	line_sen4  = ADC1_Buff[4];
	line_sen5  = ADC1_Buff[5];
	line_sen6  = ADC1_Buff[6];
	line_sen7  = ADC1_Buff[7];
	line_sen8  = ADC1_Buff[8];
	line_sen9  = ADC1_Buff[9];
	line_sen10 = ADC1_Buff[10];
	line_sen11 = ADC1_Buff[11];
}

void MotorCtrl(int16_t motorR, int16_t motorL, uint8_t stop) {

	int16_t pwmL_out,pwmR_out;

	if(motorR >= 0) {
		pwmR_out = motorR;
		MR_SET;
	}
	else {
		pwmR_out = motorR * (-1);
		MR_RESET;
	}

	if(motorL >= 0) {
		pwmL_out = motorL;
		ML_SET;
	}
	else {
		pwmL_out = motorL * (-1);
		ML_RESET;
	}

	if(pwmR_out > COUNTER_PERIOD_TIM12) pwmR_out = 839;
	if(pwmL_out > COUNTER_PERIOD_TIM12) pwmL_out = 839;

	if(stop == 1) {
		pwmR_out = 0;
		pwmL_out = 0;
	}

	__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, pwmR_out);
	__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, pwmL_out);

}

void buzzer(uint8_t bz){
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 2099);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){

	if (GPIO_Pin == GPIO_PIN_0) ; 	//left

	if (GPIO_Pin == GPIO_PIN_1) { 	//up
	  check_sens_val--;
	  if(check_sens_val < 0)  check_sens_val = CHECK_SENS_MAX;
	}

	if (GPIO_Pin == GPIO_PIN_12) {	//push
	  setup_mode++;
	  if(setup_mode >= 8) setup_mode = 0;
	}

	if (GPIO_Pin == GPIO_PIN_13) {	//down
	  check_sens_val++;
	  if(check_sens_val > CHECK_SENS_MAX) check_sens_val = 0;
	}

	if (GPIO_Pin == GPIO_PIN_14) {	//center
	  if(sw_center_state > 1) sw_center_state = 1;
	  sw_center_state ^= 1;
	}

	if (GPIO_Pin == GPIO_PIN_15) ;	//right

	//HAL_Delay(10);

}

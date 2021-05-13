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

void peripheral_init(){
	gpio_set();
	lcd_init();
	INA260_init();
	if( IMU_init() == 1 ) {
		lcd_locate(0,0);
		lcd_print("WHO_AM_I");
		lcd_locate(0,1);
		lcd_print("SUCCESS");
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
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_ALL);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, ESC_MIN);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, ESC_MIN);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, ESC_MIN);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, ESC_MIN);
	//set_timer
	HAL_TIM_Base_Start_IT(&htim6);
	HAL_TIM_Base_Start_IT(&htim7);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t *) ADC1_Buff, ADC_DATA_BUFFR_SIZE);
}

void gpio_set(){
	CS_SET;
	MR_SET;
	ML_SET;
	LED_R_SET;
	LED_G_SET;
	LED_B_SET;
}

void led_pattern(uint8_t led){
	switch(led) {
		case 0:	//無
			LED_R_SET;
			LED_G_SET;
			LED_B_SET;
			break;
		case 1:	//赤
			LED_R_RESET;
			LED_G_SET;
			LED_B_SET;
			break;
		case 2:	//緑
			LED_R_SET;
			LED_G_RESET;
			LED_B_SET;
			break;
		case 3:	//青
			LED_R_SET;
			LED_G_SET;
			LED_B_RESET;
			break;
		case 4:	//黄
			LED_R_RESET;
			LED_G_RESET;
			LED_B_SET;
			break;
		case 5:	//水
			LED_R_SET;
			LED_G_RESET;
			LED_B_RESET;
			break;
		case 6:	//紫
			LED_R_RESET;
			LED_G_SET;
			LED_B_RESET;
			break;
		case 7:	//白
			LED_R_RESET;
			LED_G_RESET;
			LED_B_RESET;
			break;
		default://無
			LED_R_SET;
			LED_G_SET;
			LED_B_SET;
			break;
	}
}

void getEncoder(void) {

	int16_t enc_tim1_ms;
	int16_t enc_tim8_ms;

	enc_tim1_ms = TIM1 -> CNT;
	enc_tim8_ms = TIM8 -> CNT;

	TIM1 -> CNT = 0;
	TIM8 -> CNT = 0;

}

//センサー基板左から		AD9 AD8 AD15 AD14 AD7 AD6   AD5 AD4 AD3 AD2 AD1 AD0
//マーカー基板左から		ADxx 	ADxx
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

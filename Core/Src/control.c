#include "control.h"

int64_t sum_zg = 0;
int16_t offset_zg = 0;
int16_t calibration_cnt;
uint8_t calibration_flag = 0;
uint8_t start_goal_flag = 0;
uint32_t log_check_adress;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {

	uint64_t enc_cnt_10ms;

	if(htim->Instance == htim6.Instance){	//1ms

		cnt_sw++;
		if(cnt_sw >= 250) cnt_sw = 30;

		getEncoder();
		read_gyro_data();
		read_accel_data();
		ADval_get();
		ADval_sum();

		if(calibration_flag == 1) {
			sum_zg += zg;
			calibration_cnt++;
		}

		if(main_pattern == 0) {
			cnt_sw++;
			maker_check = MakerSenTh(MAKERTHRESHOLD);
		}
		else if(main_pattern>=10 && main_pattern<=19) {

			if(main_pattern == 13 && second_trace_flag == 1){
				while( mm_total < mileage((float)enc_tim_total ) ) {

					if(isnan(*(float*)log_adress) != 0) {
						led_pattern(7);
						enc_cnt = 0;
						main_pattern = 14;
						break;
					}
					else mm_total += *(float*)log_adress;

					if(isnan(*(float*)plan_velo_adress) != 0) {
						led_pattern(7);
						enc_cnt = 0;
						main_pattern = 14;
						break;
					}
					else target_vel = *(float*)plan_velo_adress;

					plan_velo_adress += 0x04;
					log_adress += 0x08;
				}
				if(maker_check >= 8 && timer >= 1000) { //goal_maler_check
				//if( 1 <= maker_check && maker_check <= 3 && second_trace_flag == 0 && timer >= 800) {
					/*buf++;
					timer = 0;
					if(buf == 2){
						flash_flag = 0;
						tim_buf = timer;
						led_pattern(4);
						enc_cnt = 0;
						main_pattern = 14;
					}*/
					flash_flag = 0;
					led_pattern(4);
					enc_cnt = 0;
					main_pattern++;
					timer = 0;
				}
			}
			else if(main_pattern==14){
				if (mileage((float)enc_cnt) >= 400) {
					target_vel = 0.0f;
					led_pattern(7);
					main_pattern = 20;
					__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, 0);
					__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, 0);
				}
			}

			ErrorCheck(ERRORCHECK);
			timer++;
			CrossCheck(CROSSCHECK);
			maker_check = MakerSenTh(MAKERTHRESHOLD);//400 700
			MakerCheck(maker_check);
			posPID();
			velPID(target_vel);
			MotorCtrl((int16_t)(order_velR + order_posR), (int16_t)(order_velL + order_posL), 0);

		}
	}

	if((htim->Instance == htim7.Instance) && (flash_flag == 1)){	//10ms

		enc_cnt_10ms = (enc_tim1_cnt_10ms + enc_tim8_cnt_10ms) / 2;
		log_mm = mileage((float)enc_cnt_10ms);

		read_zg_data();
		log_zg = ((float)(zg - offset_zg) / 16.4f) * 0.01f;	//θ算出
		log_zg = fabsf(log_zg);// 絶対値
		if( crossline_flag == 1 ) log_zg = 0;

		if( log_zg == 0 ) PlanVelo[log_array] = 10000;	// θが0の場合は曲率半径は10000とする
		else PlanVelo[log_array] = log_mm / ( 2.0f * PI * ( log_zg / 360) );
		log_array++;
		//log_zg = (float)zg / 16.4f;
		FLASH_Write_Word_F(log_adress,log_zg); // 曲率半径
		log_adress += 0x04;

		FLASH_Write_Word_F(log_adress,log_mm); // 距離
		log_adress += 0x04;

		enc_tim1_cnt_10ms = 0;
		enc_tim8_cnt_10ms = 0;
	}
}

void setup(void){
	unsigned short volt_reg;

	led_pattern(setup_mode);

	switch(setup_mode) {
		case 0:	//sensor check

		if( sw_center_state == 1 ) {	//buzzer
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 1049); //MAX4199
		}
		else __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);

		switch(check_sens_val) {
			case 0:
				lcd_locate(0,0);
				lcd_printf("%4d AD0",line_sen0);
				lcd_locate(0,1);
				lcd_printf("%4d AD1",line_sen1);
				break;
			case 1:
				lcd_locate(0,0);
				lcd_printf("%4d AD2",line_sen2);
				lcd_locate(0,1);
				lcd_printf("%4d AD3",line_sen3);
				break;
			case 2:
				lcd_locate(0,0);
				lcd_printf("%4d AD4",line_sen4);
				lcd_locate(0,1);
				lcd_printf("%4d AD5",line_sen5);
				break;
			case 3:
				lcd_locate(0,0);
				lcd_printf("%4d AD6",line_sen6);
				lcd_locate(0,1);
				lcd_printf("%4d AD7",line_sen7);
				break;
			case 4:
				lcd_locate(0,0);
				lcd_printf("%4d AD8",line_sen8);
				lcd_locate(0,1);
				lcd_printf("%4d AD9",line_sen9);
				break;
			case 5:
				lcd_locate(0,0);
				lcd_printf("%4dAD10",line_sen10);
				lcd_locate(0,1);
				lcd_printf("%4dAD11",line_sen11);
				break;
			case 6:
				lcd_locate(0,0);
				lcd_printf("%4dAD10",line_sen12);
				lcd_locate(0,1);
				lcd_printf("%4dAD11",line_sen13);
				break;
			case 7:
				lcd_locate(0,0);
				lcd_printf("XG%6x",xg);
				lcd_locate(0,1);
				lcd_printf("YG%6x",yg);
				break;
			case 8:
				lcd_locate(0,0);
				lcd_printf("ZG%6x",zg);
				lcd_locate(0,1);
				lcd_printf("XA%6x",xa);
				break;
			case 9:
				lcd_locate(0,0);
				lcd_printf("YA%6x",ya);
				lcd_locate(0,1);
				lcd_printf("ZA%6x",za);
				break;
			case 10:
				lcd_locate(0,0);
				lcd_print("Encoder1");
				lcd_locate(0,1);
				lcd_printf("%8d", (int)mileage((float)enc_tim1_total));
				break;
			case 11:
				lcd_locate(0,0);
				lcd_print("Encoder2");
				lcd_locate(0,1);
				lcd_printf("%8d", (int)mileage((float)enc_tim8_total));
				break;
			case 12:
				lcd_locate(0,0);
				lcd_print("Voltage_");
				lcd_locate(0,1);
				volt_reg = INA260_read(0x02);
				lcd_printf("   %1.2fV",(float)volt_reg*0.00125f);
				break;
			case 13:
				lcd_locate(0,0);
				lcd_print("error_th");
				lcd_locate(0,1);
				lcd_printf("%8d",line_senLLL + line_senLL + line_senL + line_senR + line_senRR + line_senRRR);
				break;
			case 14:
				lcd_locate(0,0);
				lcd_print("cross_th");
				lcd_locate(0,1);
				lcd_printf("%8d", line_senLL + line_senL + line_senR + line_senRR);
				break;
			default:
				break;
			}

			break;
		case 1:
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);

			lcd_locate(0,0);
			lcd_print("test_ESC");
			lcd_locate(0,1);
			lcd_print("SW_PUSH_");

			if( sw_center_state == 1 ) {
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 2116);	//	1763(ESC_MIN) + 17.64 * 20
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 2116);
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 2116);
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 2116);
			}
			else {
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, ESC_MIN);
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, ESC_MIN);
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, ESC_MIN);
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, ESC_MIN);
			}
			break;
		case 2:
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, ESC_MIN);
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, ESC_MIN);
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, ESC_MIN);
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, ESC_MIN);

			lcd_locate(0,0);
			lcd_print("test_MD_");
			lcd_locate(0,1);
			lcd_print("SW_PUSH_");

			if( sw_center_state == 1 ) {
				__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, 400);
				__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, 400);
				MR_SET;
				ML_SET;
			}
			else {
				__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, 0);
				__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, 0);
				MR_SET;
				ML_SET;
			}
			break;
		case 3:
			__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, 0);
			__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, 0);

			lcd_locate(0,0);
			lcd_print("_erase__");
			lcd_locate(0,1);
			lcd_print("SW_PUSH_");

			if( sw_center_state == 1 ) {
				if( isnan( *(float*)start_adress_sector7 ) == 0 )  FLASH_EreaseSector(FLASH_SECTOR_7);
				if( isnan( *(float*)start_adress_sector9 ) == 0 )  FLASH_EreaseSector(FLASH_SECTOR_9);
				if( isnan( *(float*)start_adress_sector10 ) == 0 ) FLASH_EreaseSector(FLASH_SECTOR_10);
				if( isnan( *(float*)start_adress_sector11 ) == 0 ) FLASH_EreaseSector(FLASH_SECTOR_11);
			}

			break;
		case 4:
			lcd_locate(0,0);
			lcd_print("_case-4_");
			lcd_locate(0,1);
			lcd_print("________");
			break;
		case 5:
			lcd_locate(0,0);
			lcd_print("SW_PUSH");
			lcd_locate(0,1);
			lcd_print("START 1 ");
			if(sw_center_state == 1) {
				main_pattern = 10;
				timer = 0;
				enc_cnt = 0;
				sw_center_state = 0;
				velocity_pattern = 1;
				lcd_clear();
				HAL_Delay(1000);
			}
			break;
		case 6:
			lcd_locate(0,0);
			lcd_print("SW_PUSH");
			lcd_locate(0,1);
			lcd_print("START 2 ");
			if(sw_center_state == 1) {
				main_pattern = 10;
				timer = 0;
				enc_cnt = 0;
				sw_center_state = 0;
				log_check_adress = start_adress_sector10;
				if( isnan( *(float*)log_check_adress ) == 0 ) {
					second_trace_flag = 1;
					second_trace_pattern = 1;
				}
				else velocity_pattern = 2;
				lcd_clear();
				HAL_Delay(1000);
			}
			break;
		case 7:
			lcd_locate(0,0);
			lcd_print("SW_PUSH");
			lcd_locate(0,1);
			lcd_print("START 3 ");
			if(sw_center_state == 1) {
				main_pattern = 10;
				timer = 0;
				enc_cnt = 0;
				sw_center_state = 0;
				log_check_adress = start_adress_sector11;
				if( isnan( *(float*)log_check_adress ) == 0 ) {
					second_trace_flag = 1;
					second_trace_pattern = 2;
				}
				else velocity_pattern = 3;
				lcd_clear();
				HAL_Delay(1000);
			}
			break;
		default:
			break;
	}
}

void ErrorCheck(uint16_t errorthreshold) {
	if((line_senLLL + line_senLL + line_senL + line_senR + line_senRR + line_senRRR) > errorthreshold) {
		error_cnt++;
		if(error_cnt >= 5) {
			error_flag = 1;
			main_pattern = 20;
			target_vel = 0.0f;
			__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, 0);
			__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, 0);
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
		}
		if(error_cnt > 60000) error_cnt = 1000;
	}
}

void CrossCheck(uint16_t crossthreshold){

	if(crossline_flag == 0 && line_senLL + line_senL + line_senR + line_senRR < crossthreshold ) {
		crossline_flag = 1;
		enc_cnt = 0;
	}

	if(crossline_flag == 1 && mileage((float)enc_cnt) >= 90){
		crossline_flag = 0;
	}
}

uint8_t StartGoalCheck(uint8_t makerval) {
	uint8_t ret = 0;

	if( mileage((float)enc_cnt2) >= 20 && start_goal_flag == 0 && makerval == 8) {
		start_goal_flag = 1;
	}

	if( start_goal_flag == 1 ) {
		if(makerval == 0) {
			start_goal_flag = 0;
			ret = 1;
		}
		else if( (makerval &= 0x03) > 0 && (makerval &= 0x03) < 8) {
			start_goal_flag = 0;
			enc_cnt2 = 0;
		}
	}

	return ret;
}

void MakerCheck(uint8_t makerval){

	static uint8_t led = 0;
	int32_t maker_pulseL;
	int32_t maker_pulseR;
	int64_t maker_pulse_center;
	static uint16_t maker_preset = 0;
	static uint8_t maker_check_flag = 0;
	static uint8_t cmp_flag = 0;

	if((0 < makerval && makerval <= 3) && maker_check_flag == 0) {
		maker_check_flag = 1;
		enc_cnt = 0;
	}

	if((maker_check_flag == 1) && (mileage((float)enc_cnt) >= 6) && makerval == 0 ) {

		if(second_trace_flag == 1){

			maker_adress = start_adress_sector9 + maker_preset;
			cmp_flag = 1;
			led_pattern(led);

			while(cmp_flag){

				maker_pulseL = *(int32_t*)maker_adress;
				maker_adress += 0x04;
				if( isnan((float)maker_pulseL) != 0 ) cmp_flag = 0;
				maker_pulseR = *(int32_t*)maker_adress;
				maker_adress += 0x04;
				maker_pulse_center = (int64_t)(maker_pulseL + maker_pulseR) / 2;

				if((enc_tim_total > maker_pulse_center - 3000) && (enc_tim_total < maker_pulse_center + 3000)){
				//if((enc_tim_total + 3000 > maker_pulse_center) && (enc_tim_total - 3000 < maker_pulse_center)){
					enc_tim1_total = (int64_t)maker_pulseL;
					enc_tim8_total = (int64_t)maker_pulseR;
					maker_preset += 0x08;

					while( mm_total > mileage((float)maker_pulse_center) ){
						plan_velo_adress -= 0x04;
						log_adress -= 0x08;
						mm_total -= *(float*)log_adress;
						target_vel = *(float*)plan_velo_adress;
					}

					cmp_flag = 0;
					led++;
					if(led > 7) led = 0;
				}
				else if( maker_pulse_center > enc_tim_total + 8000) cmp_flag = 0;
				//else;

			}

		}
		else {

			FLASH_Write_Word_S(maker_adress,(int32_t)enc_tim1_total);
			maker_adress += 0x04;
			FLASH_Write_Word_S(maker_adress,(int32_t)enc_tim8_total);
			maker_adress += 0x04;
		}

		maker_check_flag = 0;
	}

}


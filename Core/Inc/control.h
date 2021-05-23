#ifndef control_H
#define control_H

#include "peripheral_func.h"

extern int64_t sum_zg;
extern int16_t offset_zg;
extern int16_t calibration_cnt;
extern uint8_t calibration_flag;
extern uint8_t start_goal_flag;
extern uint32_t log_check_adress;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef*);
void setup(void);
void ErrorCheck(uint16_t);
void CrossCheck(uint16_t);
uint8_t StartGoalCheck(uint8_t);
void MakerCheck(uint8_t);

#endif

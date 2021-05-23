#ifndef calculate_H
#define calculate_H

#include "peripheral_func.h"

extern float order_posR;
extern float order_posL;
extern float order_velR;
extern float order_velL;

#define DELTA_T 0.001f

float mileage(float);
float ComplementaryFilter(float, float, float, float);
void posPID(void);
void velPID(float);
void Calculation_offset_zg(void);
float Velo_Spline_Curve(float);

#endif

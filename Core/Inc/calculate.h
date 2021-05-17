#ifndef calculate_H
#define calculate_H

#include "peripheral_func.h"

#define DELTA_T 0.001f

float mileage(float);
float ComplementaryFilter(float, float, float, float);
void posPID(void);
void velPID(float);

#endif

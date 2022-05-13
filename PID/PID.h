#ifndef _PID_H_
#define _PID_H_

#include "main.h"
#include "stm32f1xx_hal.h"

float PID(float,float, float,float,float,float*,float*,float*,float*, float*);
void Constant(float *,float,float);
#endif

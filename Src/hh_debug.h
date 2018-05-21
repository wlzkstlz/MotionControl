#ifndef _HH_DEBUG_H
#define _HH_DEBUG_H

#include "stm32f1xx_hal.h"

void debugSetMaxOut(int id,float out);
float debugGetMaxOut(int id);


void debugSetLimit(int id,float limit);
float debugGetLimit(int id);

void debugSetCurOut(int id,float out);
float debugGetCurOut(int id);

void debugSetErrCode(uint8_t id);
uint8_t debugGetErrcode();


#endif
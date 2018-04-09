/*
*********************************************************************************************************
*
*	ģ������ : DAC8562 ����ģ��(��ͨ����16λDAC)
*	�ļ����� : bsp_dac8562.c
*
*	Copyright (C), 2013-2014, ���������� www.armfly.com
*
*********************************************************************************************************
*/

#ifndef _BSP_DAC8562_H
#define _BSP_DAC8562_H

#include "stm32f1xx_hal.h"

void setDacSync(uint8_t bit);
void setDin(uint8_t bit);
void setCLK(uint8_t bit);

void InitDAC8562(void);
void DAC8562_SetData(uint8_t _ch, uint16_t _dac);
void DAC8562_WriteCmd(uint32_t _cmd);

#endif

/***************************** ���������� www.armfly.com (END OF FILE) *********************************/

/*
*********************************************************************************************************
*
*	ģ������ : DAC8562/8563 ����ģ��(��ͨ����16λDAC)
*	�ļ����� : bsp_dac8562.c
*	��    �� : V1.0
*	˵    �� : DAC8562/8563ģ���CPU֮�����SPI�ӿڡ�����������֧��Ӳ��SPI�ӿں����SPI�ӿڡ�
*			  ͨ�����л���
*
*	�޸ļ�¼ :
*		�汾��  ����         ����     ˵��
*		V1.0    2014-01-17  armfly  ��ʽ����
*
*	Copyright (C), 2013-2014, ���������� www.armfly.com
*
*********************************************************************************************************
*/

#include "dac8562.h"
#include "gpio.h"
#include "stm32f1xx_hal.h"

void setDacSync(uint8_t bit)
{
	if(bit)
		HAL_GPIO_WritePin(GPIOD, ADC_SYNC_Pin, GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(GPIOD, ADC_SYNC_Pin, GPIO_PIN_RESET);
}

void setDin(uint8_t bit)
{
	if(bit)
		 HAL_GPIO_WritePin(GPIOD, ADC_IN_Pin, GPIO_PIN_SET);
	else
		 HAL_GPIO_WritePin(GPIOD, ADC_IN_Pin, GPIO_PIN_RESET);
}


void setCLK(uint8_t bit)
{
	
	if(bit)
		 HAL_GPIO_WritePin(GPIOD, ADC_SCLK_Pin, GPIO_PIN_SET);
	else
		 HAL_GPIO_WritePin(GPIOD, ADC_SCLK_Pin, GPIO_PIN_RESET);
}

/*
*********************************************************************************************************
*	�� �� ��: InitDAC8562
*	����˵��: ����STM32��GPIO��SPI�ӿڣ��������� DAC8562
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void InitDAC8562(void)
{
	setDacSync(1);	/* SYNC = 1 */

	/* Power up DAC-A and DAC-B */
	DAC8562_WriteCmd((4 << 19) | (0 << 16) | (3 << 0));
	
	/* LDAC pin inactive for DAC-B and DAC-A  ��ʹ��LDAC���Ÿ������� */
	DAC8562_WriteCmd((6 << 19) | (0 << 16) | (3 << 0));

	/* ��λ2��DAC���м�ֵ, ���0V */
	DAC8562_SetData(0, 32767);
	DAC8562_SetData(1, 32767);

	/* ѡ���ڲ��ο�����λ2��DAC������=2 ����λʱ���ڲ��ο��ǽ�ֹ��) */
	DAC8562_WriteCmd((7 << 19) | (0 << 16) | (1 << 0));
}

/*
*********************************************************************************************************
*	�� �� ��: DAC8562_WriteCmd
*	����˵��: ��SPI���߷���24��bit���ݡ�
*	��    ��: _cmd : ����
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void DAC8562_WriteCmd(uint32_t _cmd)
{
	setDacSync(0);

	for(uint8_t i = 0; i < 24; i++)
	{
		if (_cmd & 0x800000)
		{
			setDin(1);
		}
		else
		{
			setDin(0);
		}
		setCLK(1);
		_cmd <<= 1;
		setCLK(0);
	}
	setDacSync(1);
}

/*
*********************************************************************************************************
*	�� �� ��: DAC8562_SetData
*	����˵��: ����DAC��������������¡�
*	��    ��: _ch, ͨ��, 0 , 1
*		     _data : ����
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void DAC8562_SetData(uint8_t _ch, uint16_t _dac)
{
	if (_ch == 0)
	{
		/* Write to DAC-A input register and update DAC-A; */
		DAC8562_WriteCmd((3 << 19) | (0 << 16) | (_dac << 0));
	}
	else if (_ch == 1)
	{
		/* Write to DAC-B input register and update DAC-A; */
		DAC8562_WriteCmd((3 << 19) | (1 << 16) | (_dac << 0));		
	}
}

/***************************** ���������� www.armfly.com (END OF FILE) *********************************/

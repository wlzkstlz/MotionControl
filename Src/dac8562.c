/*
*********************************************************************************************************
*
*	模块名称 : DAC8562/8563 驱动模块(单通道带16位DAC)
*	文件名称 : bsp_dac8562.c
*	版    本 : V1.0
*	说    明 : DAC8562/8563模块和CPU之间采用SPI接口。本驱动程序支持硬件SPI接口和软件SPI接口。
*			  通过宏切换。
*
*	修改记录 :
*		版本号  日期         作者     说明
*		V1.0    2014-01-17  armfly  正式发布
*
*	Copyright (C), 2013-2014, 安富莱电子 www.armfly.com
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
*	函 数 名: InitDAC8562
*	功能说明: 配置STM32的GPIO和SPI接口，用于连接 DAC8562
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void InitDAC8562(void)
{
	setDacSync(1);	/* SYNC = 1 */

	/* Power up DAC-A and DAC-B */
	DAC8562_WriteCmd((4 << 19) | (0 << 16) | (3 << 0));
	
	/* LDAC pin inactive for DAC-B and DAC-A  不使用LDAC引脚更新数据 */
	DAC8562_WriteCmd((6 << 19) | (0 << 16) | (3 << 0));

	/* 复位2个DAC到中间值, 输出0V */
	DAC8562_SetData(0, 32767);
	DAC8562_SetData(1, 32767);

	/* 选择内部参考并复位2个DAC的增益=2 （复位时，内部参考是禁止的) */
	DAC8562_WriteCmd((7 << 19) | (0 << 16) | (1 << 0));
}

/*
*********************************************************************************************************
*	函 数 名: DAC8562_WriteCmd
*	功能说明: 向SPI总线发送24个bit数据。
*	形    参: _cmd : 数据
*	返 回 值: 无
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
*	函 数 名: DAC8562_SetData
*	功能说明: 设置DAC输出，并立即更新。
*	形    参: _ch, 通道, 0 , 1
*		     _data : 数据
*	返 回 值: 无
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

/***************************** 安富莱电子 www.armfly.com (END OF FILE) *********************************/

/*
 * stm32f4xx_gpio_driver.h
 *
 *  Created on: 15-Mar-2021
 *      Author: Muhammed Sijah
 */

#ifndef INC_STM32F411XX_GPIO_DRIVER_H_
#define INC_STM32F411XX_GPIO_DRIVER_H_

#include "stm32f411xx.h"

typedef struct
{
	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode;
	uint8_t GPIO_PinSpeed;
	uint8_t GPIO_PinPuPdControl;
	uint8_t GPIO_PinOPType;
	uint8_t GPIO_PinAltFunMode;
}GPIO_PinConfig_t;

typedef struct
{
	GPIO_RegDef_t *pGPIOx;
	GPIO_PinConfig_t GPIO_PinConfig;

}GPIO_Handle_t;


/*********************************************************************
 * @fn      		  - GPIO_PeriClockControl
 *
 * @brief             - This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]         - base address of the GPIO peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */

void GPIO_PeriClockControll(GPIO_RegDef_t *pGPIOx,uint8_t EnorDI)
{
	if(EnorDI == ENABLE)
		{
			if(pGPIOx == GPIOA)
			{
				GPIOA_PCLK_EN();
			}else if (pGPIOx == GPIOB)
			{
				GPIOB_PCLK_EN();
			}else if (pGPIOx == GPIOC)
			{
				GPIOC_PCLK_EN();
			}else if (pGPIOx == GPIOD)
			{
				GPIOD_PCLK_EN();
			}else if (pGPIOx == GPIOE)
			{
				GPIOE_PCLK_EN();

			}else if (pGPIOx == GPIOH)
			{
				GPIOH_PCLK_EN();

		}
		}
	else
	{
		if(pGPIOx == GPIOA)
					{
						GPIOA_PCLK_DL();
					}else if (pGPIOx == GPIOB)
					{
						GPIOB_PCLK_DL();
					}else if (pGPIOx == GPIOC)
					{
						GPIOB_PCLK_DL();
					}else if (pGPIOx == GPIOD)
					{
						GPIOB_PCLK_DL();
					}else if (pGPIOx == GPIOE)
					{
						GPIOB_PCLK_DL();

					}else if (pGPIOx == GPIOH)
					{
						GPIOB_PCLK_DL();
					}
    }
}



void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInt(GPIO_RegDef_t *pGPIOx);

/* gpio config input and output */

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber,uint8_t Value);
void GPIO_WritetoOutPort(GPIO_RegDef_t *pGPIOx,uint16_t Value);
void GPIO_ToggleOutputPIn(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber);

/* IRQ Config */

void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IQRPriority,uint8_t ENorDI);
void GPIO_IRQHandling(uint8_t PinNumber);













#endif /* INC_STM32F411XX_GPIO_DRIVER_H_ */

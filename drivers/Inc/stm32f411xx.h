/*
 * stm32f411xx.h
 *
 *  Created on: Mar 12, 2021
 *      Author: Muhammed Sijah
 */

#ifndef INC_STM32F411XX_H_
#define INC_STM32F411XX_H_

#include<stdint.h>
#define __vo  volatile

#define FLASH_BASEADDR   0X08000000U //FLASH BASE ADRESS
#define ROM_BASEADDR     0X1FFF0000U //ROM BASE ADRESS
#define SRAM_BASEADDR    0X20000000U //SSRAM BASE ADRESS

/* AHBx AND APBx */


#define PERIPH_BASEADDR     0X40000000U
#define APB1PERIPH_BASEADDR PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR 0X40010000U
#define AHB1PERIPH_BASEADDR 0X40020000U
#define AHB2PERIPH_BASEADDR 0X50000000U
#define RCC_BASEADDR (AHB1PERIPH_BASEADDR+0X3800)
/*BASE ADRES OF PERIPHARLS HANGING ON AHB1 BUSES*/

#define GPIOA_BASEADDR  (AHB1PERIPH_BASEADDR)
#define GPIOB_BASEADDR  (AHB1PERIPH_BASEADDR+0X0400)
#define GPIOC_BASEADDR  (AHB1PERIPH_BASEADDR+0X0800)
#define GPIOD_BASEADDR  (AHB1PERIPH_BASEADDR+0X0C00)
#define GPIOE_BASEADDR  (AHB1PERIPH_BASEADDR+0X1000)
#define GPIOH_BASEADDR  (AHB1PERIPH_BASEADDR+0X1C00)

/*BASE ADRES OF PERIPHARLS HANGING ON APB1 BUSES*/

#define I2C1_BASEADDR   (APB1PERIPH_BASEADDR+0X5400)
#define I2C2_BASEADDR   (APB1PERIPH_BASEADDR+0X5800)
#define I2C3_BASEADDR   (APB1PERIPH_BASEADDR+0X5C00)
#define SPI2_BASEADDR   (APB1PERIPH_BASEADDR+0X3800)
#define SPI3_BASEADDR   (APB1PERIPH_BASEADDR+0X3C00)
#define USART2_BASEADDR (APB1PERIPH_BASEADDR+0X4400)


/*BASE ADRES OF PERIPHARLS HANGING ON APB2 BUSES*/

#define SPI1_BASEADDR   (APB2PERIPH_BASEADDR+0X3000)
#define SPI4_BASEADDR   (APB2PERIPH_BASEADDR+0X3400)
#define SPI5_BASEADDR   (APB2PERIPH_BASEADDR+0X5000)
#define USART1_BASEADDR (APB2PERIPH_BASEADDR+0X1000)
#define USART6_BASEADDR (APB2PERIPH_BASEADDR+0X1400)
#define EXT1_BASEADDR   (APB2PERIPH_BASEADDR+0X3C00)



/* Peripheral gpio structure */

typedef struct{
	__vo uint32_t MODER;
	__vo uint32_t OTYPER;
	__vo uint32_t OSPEEDR;
	__vo uint32_t PUPDR;
	__vo uint32_t IDR;
	__vo uint32_t ODR;
	__vo uint32_t BSRR;
	__vo uint32_t LCKR;
	__vo uint32_t AFR[2];
}GPIO_RegDef_t;

/* Peripheral RCC structure */
typedef struct{
	__vo uint32_t CR;
	__vo uint32_t PLLCFGR;
	__vo uint32_t CFGR;
	__vo uint32_t CIR;
	__vo uint32_t AHB1RSTR;
	__vo uint32_t AHB2RSTR;
	 uint32_t RESERVERD0[2];
	__vo uint32_t APB1RSTR;
	__vo uint32_t APB2RSTR;
	uint32_t RESERVERD1[2];
	__vo uint32_t AHB1ENR;
	__vo uint32_t AHB2ENR;
	uint32_t RESERVERD2[2];
	__vo uint32_t APB1ENR;
	__vo uint32_t APB2ENR;
	uint32_t RESERVERD3[2];
	__vo uint32_t AHB1LPENR;
	__vo uint32_t AHB2LPENR;
	uint32_t RESERVERD4[2];
	__vo uint32_t APB1LPENR;
	__vo uint32_t APB2LPENR;
	uint32_t RESERVERD5[2];
	__vo uint32_t BDCR;
	__vo uint32_t CSR;
	uint32_t RESERVERD6[2];
	__vo uint32_t SSCGR;
	__vo uint32_t PLLI2SCFGR;

}RCC_RegDef_t;





/*PERIPHRAL DEFINATIONS GPIO */


#define GPIOA    ((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB    ((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC    ((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD    ((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE    ((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOH    ((GPIO_RegDef_t*)GPIOH_BASEADDR)

/*  PERIPHRAL DEFINATIONS RCC */
#define RCC      ((RCC_RegDef_t*)+ RCC_BASEADDR )

/* clock enable definition GPIO */

#define GPIOA_PCLK_EN() (RCC->AHB1ENR |= (1<<0))
#define GPIOB_PCLK_EN() (RCC->AHB1ENR |= (1<<1))
#define GPIOC_PCLK_EN() (RCC->AHB1ENR |= (1<<2))
#define GPIOD_PCLK_EN() (RCC->AHB1ENR |= (1<<3))
#define GPIOE_PCLK_EN() (RCC->AHB1ENR |= (1<<4))
#define GPIOH_PCLK_EN() (RCC->AHB1ENR |= (1<<7))

/* ENABLE CLOCK DEFFINITION I2C-*/
#define I2C1_PCLK_EN()  (RCC->APB1ENR |=(1<<21))
#define I2C2_PCLK_EN()  (RCC->APB1ENR |=(1<<22))
#define I2C3_PCLK_EN()  (RCC->APB1ENR |=(1<<23))

/* ENABLE CLOCK DEFINATION SPI*/
#define SPI1_PCLK_EN()  (RCC->APB2ENR |=(1<<12))
#define SPI2_PCLK_EN()  (RCC->APB1ENR |=(1<<14))
#define SPI3_PCLK_EN()  (RCC->APB1ENR |=(1<<15))
#define SPI4_PCLK_EN()  (RCC->APB2ENR |=(1<<13))
#define SPI5_PCLK_EN()  (RCC->APB2ENR |=(1<<20))

/* ENABLE CLOCK DEFINATION UART*/
#define USART1_PCLK_EN() (RCC->APB2ENR |=(1<<4))
#define USART2_PCLK_EN() (RCC->APB1ENR |=(1<<17))
#define USART6_PCLK_EN() (RCC->APB2ENR |=(1<<5))

/* DISABLE CLOCK definition GPIO */
#define GPIOA_PCLK_DL() (RCC->AHB1ENR &= ~(1<<0))
#define GPIOB_PCLK_DL() (RCC->AHB1ENR &= ~(1<<1))
#define GPIOC_PCLK_DL() (RCC->AHB1ENR &= ~(1<<2))
#define GPIOD_PCLK_DL() (RCC->AHB1ENR &= ~(1<<3))
#define GPIOE_PCLK_DL() (RCC->AHB1ENR &= ~(1<<4))
#define GPIOH_PCLK_DL() (RCC->AHB1ENR &= ~(1<<7))













#endif /* INC_STM32F411XX_H_ */

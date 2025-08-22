

#ifndef __FOC_H__
#define __FOC_H__
#endif

#include "stm32f4xx_hal.h"

#define PWM_Period 100
#define _PI 3.14159265359
#define _PI_2 1.57079632679
#define _PI_3 1.0471975512
#define _2PI 6.28318530718
#define _3PI_2 4.71238898038
#define _PI_6 0.52359877559
#define _SQRT3 1.73205080757
#define _round(x) ((x)>=0?(long)((x)+0.5):(long)((x)-0.5))
#define Tr 0.026; //us  T=50us

extern float voltage_power_supply;

extern TIM_HandleTypeDef htim1;

float CurrentCircule_Id(float Ia, float Ib, float Ic, float angle_el);
float CurrentCircule_Iq(float Ia, float Ib, float Ic, float angle_el);

void setPhaseVoltage(float Uq, float Ud, float angle_el);

float _cos(float a);
float _sin(float a);

//typedef struct{
//	SPI_HandleTypeDef *pSPI_Handle;
//	GPIO_TypeDef* PORT_GPIOx;
//	uint16_t CS_GPIO_Pin;
//	uint16_t SPI_RX_Data[10];
//	uint16_t SPI_TX_Data[10];
//	uint8_t SPI_TX_Flag;
//	uint8_t SPI_RX_Flag;
//} SPIStruct;
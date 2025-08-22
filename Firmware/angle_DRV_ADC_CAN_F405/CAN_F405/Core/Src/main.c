/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "can.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include <math.h>
#include "lcd114.h"
#include "AS5600.h"
#include "drv8323.h"
#include "FOC.h"
#include "PID.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#define Duty 40
//#define A0 __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,0)
//#define B0 __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,0)
//#define C0 __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,0)
//#define A1 __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,Duty)
//#define B1 __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,Duty)
//#define C1 __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,Duty)
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
float angle_sp=0;                         //�Ƕȸ���
float vel_sp=0;                           //ת�ٸ���
float tor_sp=0;                           //���ظ���

float angle_el=0;                         //��Ƕ�
float angle=0;                            //��е�Ƕ�/56

int p=14;               //���������

extern float vel_LPF;    //PID����
extern float tor_q;
extern float Iq;
extern float Id;
extern float Uq_set;
extern float Ud_set;
extern float vel_c;
extern float Ts;

extern float integral_Iq_prev;
extern float integral_vel_prev;

extern float Power;      //ADC����
extern float Ia;
extern float Ib;
extern float Ic;

float warebuf[8];

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
//int fputc(int c,FILE *sream)
//{
//	HAL_UART_Transmit(&huart1,(unsigned char *)&c,1,1000);
//	return 1;
//}
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
float as5600GetAngle();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
  static CAN_TxHeaderTypeDef TxHeader;
  static CAN_RxHeaderTypeDef RxHeader;
  uint8_t TxData[8];
	uint8_t RxData[8];
	
	uint32_t TxMailbox;

	float current;
  
	int16_t set_speed;
	int16_t set_angle;
	float set_current;
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan1)                   //CAN���ݽ��ջص�
{
	HAL_CAN_GetRxMessage(hcan1,CAN_RX_FIFO0,&RxHeader,RxData);
	if(RxHeader.StdId == 0x0201)                                                    //ת��
	{
		if(RxData[0] == 0) vel_sp = (16*16*16*RxData[1]+16*16*RxData[2]+16*RxData[3]+RxData[4])*2*3.14/60;
		else if (RxData[0] == 1) vel_sp = -(16*16*16*RxData[1]+16*16*RxData[2]+16*RxData[3]+RxData[4])*2*3.14/60;
			DRV832x_initSPIInterface(GPIOA, GPIO_PIN_4, &hspi1, GPIOA, GPIO_PIN_5);

	}
	else if(RxHeader.StdId == 0x0202)                                               //�Ƕ�
	{
		if(RxData[0] == 0) angle_sp = 16*16*RxData[1]+16*RxData[2]+RxData[3]; 
		else if (RxData[0] == 1) angle_sp = -(16*16*RxData[1]+16*RxData[2]+RxData[3]); 
			DRV832x_initSPIInterface(GPIOA, GPIO_PIN_4, &hspi1, GPIOA, GPIO_PIN_5);
	}
	else if(RxHeader.StdId == 0x0203)                                               //����
	{
		if(RxData[0] == 0) tor_sp = RxData[1]+0.01*(16*RxData[2]+RxData[3]);
		else if (RxData[0] == 1) tor_sp = -(RxData[1]+0.01*(16*RxData[2]+RxData[3]));
		DRV832x_initSPIInterface(GPIOA, GPIO_PIN_4, &hspi1, GPIOA, GPIO_PIN_5);
	}
	else if(RxHeader.StdId == 0x0204)
	{
		DRV832x_disable(GPIOA, GPIO_PIN_5);
		angle_sp=0;                         
    vel_sp=0;                          
    tor_sp=0;
		integral_Iq_prev=0;
		integral_vel_prev=0;
	}		
}





/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN1_Init();
  MX_SPI3_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
	HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);

  DRV832x_enable_gd();
	
//	DRV832x_disable(GPIOA, GPIO_PIN_5);
//	delay_us(40);
//  DRV832x_enable(GPIOA, GPIO_PIN_5);
  //DRV832x_initSPIInterface(GPIOA, GPIO_PIN_4, &hspi1, GPIOA, GPIO_PIN_5);
	DRV832x_write_DCR(DIS_CPUV_EN, DIS_GDF_EN, OTW_REP_DIS, PWM_MODE_6X, PWM_1X_COM_SYNC , PWM_1X_DIR_0, 0x0, 0x0, 0x1);
	delay_us(100);
	DRV832x_write_HSR(LOCK_OFF, IDRIVEP_HS_1000MA, IDRIVEN_HS_380MA);
	delay_us(100);
  DRV832x_write_LSR(0x1, TDRIVE_1000NS, IDRIVEP_LS_1000MA, IDRIVEN_LS_380MA);
	delay_us(100);
  DRV832x_write_OCPCR(TRETRY_4MS, DEADTIME_50NS, OCP_RETRY, OCP_DEG_8US, VDS_LVL_0_06);
	delay_us(100);
  DRV832x_write_CSACR(CSA_FET_SP, VREF_DIV_1, LS_REF_SH_SP , CSA_GAIN_40, DIS_SEN_EN, CSA_CAL_NROMAL, CSA_CAL_NROMAL, CSA_CAL_NROMAL, SEN_LVL_1_0);
 
	DRV832x_disable_gd();
	
//	TxHeader.DLC = 2;
//  TxHeader.IDE = CAN_ID_STD;
//	TxHeader.RTR = CAN_RTR_DATA;
//	TxHeader.StdId = 0x446;
	
//	TxData[0] = 100;
//	TxData[1] = 10;
	CAN1_Filter_init();
	HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);
  
	HAL_ADC_Start_IT(&hadc1);
	//HAL_ADC_Start(&hadc1);
	HAL_ADCEx_InjectedStart_IT(&hadc1);
	HAL_TIM_Base_Start_IT(&htim2);
  //HAL_TIM_Base_Start_IT(&htim3);
	//HAL_TIM_Base_Start_IT(&htim4);
	//HAL_ADC_Start_DMA(&hadc1,&adc_current[3],3);
	
	Lcd_Init();
	LCD_Clear(BLACK);
  BACK_COLOR=BLACK;
	as5600Init();
		voltage_power_supply=24;
	LCD_ShowString(220,10,"V",YELLOW);
	LCD_ShowString(10,30,"Speed:",RED);
	LCD_ShowString(10,45,"Angle:",BLUE);
	//LCD_ShowString(10,90,"Current:",GREEN);
	LCD_ShowString(10,75,"Uq_set:",CYAN);
		LCD_ShowString(10,90,"Ud_set:",CYAN);
	LCD_ShowString(10,60,"Tor_sp:",GREEN);
	LCD_ShowString(10,105,"Id:",CYAN);
	
	//HAL_UART_Init(&huart1);
	
  /* USER CODE END 2 */
 // for(int i=0; i<1000; i++)     //��Ƕȳ�ʼ��
	//{
	//	setPhaseVoltage(0,2,0);
	// }
	
	
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
		//HAL_UART_Transmit(&huart1,"FUCK",6,0xFFFF);
    LCD_ShowNum1(180,10,Power,4,YELLOW);
    //setPhaseVoltage(5,0,angle_el);
//		angle_el=angle_el+0.1;
		//angle=as5600GetAngle();
		//angle_el=angle*p;  	
		LCD_ShowNum1(70,45,fabsf(angle_sp),4,WHITE);		
		LCD_ShowNum1(70,30,fabsf(vel_sp*9.56-10),5,WHITE);
		LCD_ShowNum1(80,60,fabsf(tor_sp),6,WHITE);
		LCD_ShowNum1(30,10,fabsf(angle*56),7,WHITE);
		LCD_ShowNum1(180,45,fabsf(vel_c),4,WHITE);         //vel_c
		
		LCD_ShowNum1(180,30,fabsf(vel_LPF*9.56),5,WHITE);
//	LCD_ShowNum1(80,60,Ia,5,WHITE);
	  LCD_ShowNum1(180,60,fabsf(Iq),4,WHITE);
		//LCD_ShowNum1(180,75,fabsf(Id),4,WHITE);
//		
		LCD_ShowNum1(80,75,fabsf(Uq_set),6,WHITE);
		LCD_ShowNum1(80,90,fabsf(Ud_set),6,WHITE);
		LCD_ShowNum1(30,105,Id,5,GRAY);
		
		LCD_ShowNum1(180,75,Ia,5,GRAY);
		LCD_ShowNum1(180,90,Ib,5,GRAY);
		LCD_ShowNum1(180,105,Ic,5,GRAY);
		if(Ia<0) LCD_ShowString(170,75,"-",WHITE);
		else LCD_ShowString(170,75," ",WHITE);
		if(Ib<0) LCD_ShowString(170,90,"-",WHITE);
		else LCD_ShowString(170,90," ",WHITE);
		if(Ic<0) LCD_ShowString(170,105,"-",WHITE);
		else LCD_ShowString(170,105," ",WHITE);
		
		if(vel_sp<0) LCD_ShowString(60,30,"-",WHITE);
		else LCD_ShowString(60,30," ",WHITE);
		if(Uq_set<0) LCD_ShowString(70,75,"-",WHITE);
		else LCD_ShowString(70,75," ",WHITE);
		if(angle_el<0) LCD_ShowString(10,10,"-",WHITE);
		else LCD_ShowString(10,10," ",WHITE);
		if(vel_LPF<0) LCD_ShowString(160,30,"-",WHITE);
		else LCD_ShowString(160,30," ",WHITE);
		if(Iq<0) LCD_ShowString(160,60,"-",WHITE);
		else LCD_ShowString(160,60," ",WHITE);
		if(tor_sp<0) LCD_ShowString(70,60,"-",WHITE);
		else LCD_ShowString(70,60," ",WHITE);
		//HAL_Delay(1);
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */


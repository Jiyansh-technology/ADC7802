/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define DEV_ADD ((0x2A)<<1)
#define REG_ADD 0
#define WRT_DAT 1
#define ADC_CTRL_REG  0x00
#define ADC_CTRL_REG1  0x01
#define ADC_RESULT_ADDRESS  0x12
#define ADC_RESET  0x01
#define ADC_START  0x02
#define ADC_INITIALIZE  0x3E

#define ADC_GAIN_CALIB  0x06
#define ADC_OFFSET_CALIB  0x03

 uint8_t i2cDataBuff_W[10];
//uint8_t i2cDataBuff_R[10];
//uint8_t i2cDataBuff2_R[10];
//uint8_t ctrlReg2Val;
//uint32_t gainCalib;
//uint32_t offsetCalib;

//----------Multiplexer------------------------//
#define I2C_TCA9548A_ADDR			((0x70)<<1)	// write = 0xE0, read = 0xE1

#define CMD_TCA9548A_OFF			0x00
#define CMD_TCA9548A_CH0			0x01
#define CMD_TCA9548A_CH1			0x02
#define CMD_TCA9548A_CH2			0x04
#define CMD_TCA9548A_CH3			0x08
#define CMD_TCA9548A_CH4			0x10
#define CMD_TCA9548A_CH5			0x20
#define CMD_TCA9548A_CH6			0x40
#define CMD_TCA9548A_CH7			0x80


union Data
{
	unsigned long int value;
	float cf1;
}multiplication_factor,calibration_facotr,BaseCount;

union CAP_CT
{
	unsigned char pac[4];
	unsigned long int ct;
	float c_pg;
};
union CAP_CT  count;

float diff,wt_f=0;
long int CurrentCount,loadd,weight,editt,c;
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

unsigned long int read7802(int channel)
{
	int32_t test_value = 0;
	HAL_StatusTypeDef ret = 0;
	unsigned char tx[2],x;
	unsigned long int s=0;

	i2cDataBuff_W[0]=(CMD_TCA9548A_OFF| channel);
	ret = HAL_I2C_Master_Transmit(&hi2c1,I2C_TCA9548A_ADDR,&i2cDataBuff_W[REG_ADD],1,HAL_MAX_DELAY);
	HAL_Delay(50);


	for(x=0;x<1;x++)
	{
		tx[0]=0x12;
		HAL_I2C_Master_Transmit(&hi2c1,DEV_ADD,&tx[0],1,5);
		HAL_I2C_Master_Receive(&hi2c1,DEV_ADD,&count.pac[2],1,5);

		tx[0]=0x13;
		HAL_I2C_Master_Transmit(&hi2c1,DEV_ADD,&tx[0],1,5);
		HAL_I2C_Master_Receive(&hi2c1,DEV_ADD,&count.pac[1],1,5);

		tx[0]=0x14;
		HAL_I2C_Master_Transmit(&hi2c1,DEV_ADD,&tx[0],1,5);
		HAL_I2C_Master_Receive(&hi2c1,DEV_ADD,&count.pac[0],1,5);

		count.pac[3]=0;
		if (count.ct>0x7FFFFF)
		{
			count.ct|=0xFF800000;
		}
		s=s+count.ct;
		s=s+test_value;
	}
	s=s/1;
	return(s);
}

void I2C_channel_initADC(int channel)
{
	HAL_StatusTypeDef ret = 0;
	unsigned char tx[2];

	i2cDataBuff_W[0]=(CMD_TCA9548A_OFF| channel);
	//	  i2cDataBuff_W[WRT_DAT]=ADC_RESET;
	ret = HAL_I2C_Master_Transmit(&hi2c1,I2C_TCA9548A_ADDR,&i2cDataBuff_W[REG_ADD],1,HAL_MAX_DELAY);
	HAL_Delay(100);

	tx[0]=0x00;
	tx[1]=0x01;
	HAL_I2C_Master_Transmit(&hi2c1,DEV_ADD,tx,2,5);
	HAL_Delay(50);

	tx[0]=0x00;
	tx[1]=0x02;
	HAL_I2C_Master_Transmit(&hi2c1,DEV_ADD,tx,2,5);
	HAL_Delay(50);


	tx[0]=0x00;
	tx[1]=0x96;
	HAL_I2C_Master_Transmit(&hi2c1,DEV_ADD,tx,2,5);
	HAL_Delay(50);


	tx[0]=0x01;
	tx[1]=0x07;
	HAL_I2C_Master_Transmit(&hi2c1,DEV_ADD,tx,2,5);
	HAL_Delay(50);


	tx[0]=0x15;
	tx[1]=0x30;
	HAL_I2C_Master_Transmit(&hi2c1,DEV_ADD,tx,2,5);
	HAL_Delay(50);


	tx[0]=0x1c;
	tx[1]=0x90;
	HAL_I2C_Master_Transmit(&hi2c1,DEV_ADD,tx,2,5);
	HAL_Delay(50);


	tx[0]=0x1b;
	tx[1]=0x60;
	HAL_I2C_Master_Transmit(&hi2c1,DEV_ADD,tx,2,5);
	HAL_Delay(50);


	tx[0]=0x02;
	tx[1]=0x03;
	HAL_I2C_Master_Transmit(&hi2c1,DEV_ADD,tx,2,5);
	HAL_Delay(50);


	tx[0]=0x11;
	tx[1]=0x30;
	HAL_I2C_Master_Transmit(&hi2c1,DEV_ADD,tx,2,5);
	HAL_Delay(50);

	tx[0]=0x03;
	tx[1]=0x01;
	HAL_I2C_Master_Transmit(&hi2c1,DEV_ADD,tx,2,5);
	HAL_Delay(50);

	tx[0]=0x04;
	tx[1]=0x86;
	HAL_I2C_Master_Transmit(&hi2c1,DEV_ADD,tx,2,5);
	HAL_Delay(50);

	tx[0]=0x05;
	tx[1]=0xa0;
	HAL_I2C_Master_Transmit(&hi2c1,DEV_ADD,tx,2,5);
	HAL_Delay(50);

}
long int avg(int channel)
{
	int i;
	unsigned long int ADC_Counts,Avg_ADC_Counts=0;
	for(i=0;i<30;i++)
	{
		ADC_Counts=read7802(channel);
		Avg_ADC_Counts=Avg_ADC_Counts+ADC_Counts;
	}
	return(Avg_ADC_Counts/300);
}
void calibration(int channel)
{
	unsigned long int val=0;
	//counts without any load
	//for(i=0;i<20;i++)
	BaseCount.value=avg(channel);
	//display "LOAD" on screen
	//while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9)!=0);
	HAL_Delay(500);
	while(1)
	{
		if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9)==0)
			break;
	}
	//while(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_14)!=0);
	//weight in grams for calibration read from display(user)
	loadd=1000;
	//counts after keeping load
	//for(i=0;i<20;i++)
	CurrentCount=avg(channel);
	//calibration factor(counts per gram)
	val=CurrentCount-BaseCount.value;
	calibration_facotr.cf1=(float)val/(float)loadd;
	//save calibration factor in memory for weight calculation
	//save();
}
void calculation(int channel)
{

	CurrentCount=avg(channel);
	if(CurrentCount>BaseCount.value)
	{
		diff=CurrentCount-BaseCount.value;
		//neg_f=0;
	}
	else
	{
		diff=BaseCount.value-CurrentCount;
		//neg_f=1;
	}
	weight=diff/calibration_facotr.cf1;
	wt_f=(float)(diff)/(calibration_facotr.cf1);
	//	wt_f=wt_f-weight;
	//	weight=weight/acc;
	//	weight=weight*acc;
}
void Uart_PutNumber(UART_HandleTypeDef *huart, uint32_t x)
{
	char value[10]; //a temp array to hold results of conversion
	int i = 0; //loop index
	uint8_t data[] = "WEIGHT :";
	uint8_t data1[] = "\r\n";

	do
	{
		value[i++] = (char)(x % 10) + '0'; //convert integer to character
		x /= 10;
	} while(x);
	HAL_UART_Transmit (&huart1, data, sizeof (data), 10);
	while(i) //send data
	{
		HAL_UART_Transmit(huart, (uint8_t*)&value[--i], sizeof(value[--i]), HAL_MAX_DELAY);
	}
	HAL_UART_Transmit (&huart1, data1, sizeof (data1), 10);
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

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_I2C1_Init();
	MX_USART1_UART_Init();
	/* USER CODE BEGIN 2 */
	I2C_channel_initADC(CMD_TCA9548A_CH0);
	//I2C_channel_initADC(CMD_TCA9548A_CH1);
	if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9)!=0)
	{
		//while (1)
		{
//			BaseCount.value=read7802(CMD_TCA9548A_CH0);
//			Uart_PutNumber(&huart1, BaseCount.value);
			calibration(CMD_TCA9548A_CH0);
		}

	}
	/* USER CODE END 2 */
	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		calculation(CMD_TCA9548A_CH0);
		Uart_PutNumber(&huart1, wt_f);
		HAL_Delay(20);
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

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void)
{

	/* USER CODE BEGIN I2C1_Init 0 */

	/* USER CODE END I2C1_Init 0 */

	/* USER CODE BEGIN I2C1_Init 1 */

	/* USER CODE END I2C1_Init 1 */
	hi2c1.Instance = I2C1;
	hi2c1.Init.ClockSpeed = 100000;
	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void)
{

	/* USER CODE BEGIN USART1_Init 0 */

	/* USER CODE END USART1_Init 0 */

	/* USER CODE BEGIN USART1_Init 1 */

	/* USER CODE END USART1_Init 1 */
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 115200;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

	/*Configure GPIO pin : PC13 */
	GPIO_InitStruct.Pin = GPIO_PIN_13;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin : PB9 */
	GPIO_InitStruct.Pin = GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

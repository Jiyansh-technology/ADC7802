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
#include "i2c.h"
#include "usart.h"
#include "gpio.h"
#include <stdio.h>
#include <string.h>
#include <STM32f103_FLASH_RW.h>
#include <math.h>
#include "ftoa.h"

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

#define PAGE_0 0
#define PAGE_1 1
#define PAGE_2 2
#define PAGE_3 3
#define PAGE_4 4
#define PAGE_5 5
#define PAGE_6 6
#define PAGE_7 7
#define PAGE_8 8
#define PAGE_9 9

#define VEHICLE_COMPANY		0
#define ENGINEER_NAME		1
#define MODEL_NAME			2
#define CLIENT_NAME			3
#define VEHICLE_REG_NO		4
#define SETTING				5
#define PRINT				6
#define SAVE				7

#define FLASH_ADDRESS 0x08007C00

uint8_t i2cDataBuff_W[10];
//uint8_t i2cDataBuff_R[10];
//uint8_t i2cDataBuff2_R[10];
//uint8_t ctrlReg2Val;
//uint32_t gainCalib;
//uint32_t offsetCalib;
/*********************************/
int len=0;
char usartTx[50];
char usartRx[50];
uint8_t cmd[3]={0xFF,0xFF,0xFF};
uint8_t sRet=0;
uint8_t details_type=VEHICLE_COMPANY;
char vahicle_company[10]={0};
char engineer_name[10]={0};
char model_name[10]={0};
char client_name[10]={0};
char vahicle_no[10]={0};
uint8_t data_update_flag=1;

char datatest[10]={0};
char rxData[50]={0};
uint8_t startFlag=0;
uint8_t stopFlag=0;
uint8_t index1=0;
uint8_t Data_Received_Ok = RESET;
uint8_t current_page=PAGE_1;


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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
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

uint8_t Nextion_Change_Screen(uint8_t screen_no)
{
	//current_page= screen_no;
	len= sprintf(usartTx,"page %d",screen_no);
	sRet = HAL_UART_Transmit(&huart1,(uint8_t*)usartTx,len,1000);
	sRet = HAL_UART_Transmit(&huart1,(uint8_t*)cmd,sizeof(cmd),1000);
	sRet = HAL_UART_Transmit(&huart2,(uint8_t*)usartTx,len,1000);
	HAL_Delay(100);
	return sRet;
}

void calibration1(int channel)
{
	unsigned long int val=0;
	int index=0,x=0,data1=0;
	int data[10]={0};
	//counts without any load
	//for(i=0;i<20;i++)
	BaseCount.value=avg(channel);
	HAL_Delay(50);
	while(1)
	{
		if(rxData[0] == 'T')
		{
			//weight in grams for calibration read from display(user)
			HAL_Delay(50);
			while(rxData[index] !='\0')
			{
				data[index]=rxData[index]-48;
				index++;
			}

			for(int j=index-1;j >= 0;j--)
			{
				data1=data[x] *pow(10,j);
				loadd = loadd+data1;
				x++;
			}

			//Nextion_Change_Screen(9);
			break;
		}


	}
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
	//wt_f=(float)(diff)/(calibration_facotr.cf1);
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

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance ==USART1)
	{
		// HAL_UART_Receive_IT(&huart1, (uint8_t*)usartRx, 2);
		//  len= sprintf(datatest,"shyamRX");
		//HAL_UART_Transmit(&huart2,(uint8_t*)usartRx,1,10);

		if(usartRx[0] == '*')
		{
			startFlag = SET;
			//stopFlag = RESET;
			index1=0;
			goto skip;
		}
		if(usartRx[0] == '#')
		{
			startFlag = RESET;
			stopFlag = SET;
			//index1=0;
		}
		if(startFlag == SET)
		{
			rxData[index1] = usartRx[0];
			index1++;
		}
		//	HAL_UART_Transmit(&huart2,(uint8_t*)usartRx,1,10);
		if(stopFlag == SET)
		{
			stopFlag = RESET;
			index1=0;
			len= sprintf(datatest,rxData);
			HAL_UART_Transmit(&huart2,(uint8_t*)datatest,len,10);
			Data_Received_Ok = SET;
		}
		skip:
		HAL_UART_Receive_IT(&huart1, (uint8_t*)usartRx, 1);
		//HAL_Delay(100);
	}

	//	len= sprintf(usartTx,"t1.txt=\"200\"");
	//
	//	  	 HAL_UART_Transmit(&huart1,(uint8_t*)usartTx,len,1000);
	//	  	 HAL_UART_Transmit(&huart1,(uint8_t*)cmd,sizeof(cmd),1000);
}

void settingMenu()
{

}

uint8_t Nextion_Send_Data(char *textName,char *textData)
{
	len= sprintf(usartTx,"%s.txt=\"%s\"",textName,textData);
	sRet = HAL_UART_Transmit(&huart1,(uint8_t*)usartTx,len,1000);
	sRet = HAL_UART_Transmit(&huart1,(uint8_t*)cmd,sizeof(cmd),1000);
	sRet = HAL_UART_Transmit(&huart2,(uint8_t*)usartTx,len,1000);
	HAL_Delay(100);

	return sRet;
}


void print_data()
{
	char printData[1000]={0};
	uint8_t cmd[3]={0x1B,0x21,0x10};
	HAL_UART_Transmit(&huart2,(uint8_t*)cmd,sizeof(cmd),10);
	len= sprintf(usartTx,"        Precise Technology\n");
	HAL_UART_Transmit(&huart2,(uint8_t*)usartTx,len,100);
	HAL_Delay(50);
	cmd[0]=0x1B;
	cmd[1]=0x21;
	cmd[2]=0x00;
	HAL_UART_Transmit(&huart2,(uint8_t*)cmd,sizeof(cmd),100);
	HAL_Delay(50);
	len= sprintf(printData,"        Mob. No.- 7755906028\n "
			"Date 	04/03/2022 Time: 04:54:48\n"
			"--------------------------------\n"
			"    Client Name: %s\n"
			"    Vehicle Reg: %s\n"
			"    Vehicle Company: %s\n"
			"    Model: %s\n"
			"    Test Engg.: %s\n"
			"    Axial weight: \n"
			"    1) \n"
			"    2) \n"
			"    3)	\n"
			"--------------------------------\n"
			"Total weight: KG\n"
			"----------------------------------\n"
			"         Thank you\n"
			"         Visit Again\n"
			" \n"
			" \n"
			,client_name,vahicle_no,vahicle_company,model_name,engineer_name); //0x0801FC00
	sRet = HAL_UART_Transmit(&huart2,(uint8_t*)printData,len,1000);

}

void pendrive_save_data()
{
	char printData[1000]={0};

	HAL_Delay(100);
	len= sprintf(printData, "        Precise Technology\n"
			"        Mob. No.- 7755906028\n "
			"Date 	04/03/2022 Time: 04:54:48\n"
			"--------------------------------\n"
			"    Client Name: %s\n"
			"    Vehicle Reg: %s\n"
			"    Vehicle Company: %s\n"
			"    Model: %s\n"
			"    Test Engg.: %s\n"
			"    Axial weight: \n"
			"    1) \n"
			"    2) \n"
			"    3)	\n"
			"--------------------------------\n"
			"Total weight: KG\n"
			"----------------------------------\n"
			"         Thank you\n"
			"         Visit Again\n"
			" \n"
			" \n"
			,client_name,vahicle_no,vahicle_company,model_name,engineer_name); //0x0801FC00
	sRet = HAL_UART_Transmit(&huart3,(uint8_t*)printData,len,1000);
	HAL_Delay(100);
}
void Edit_Details(char *Data)
{
	int i=0;
	if(Data[0] == 'v') i=1;
	else if(Data[0] == 'e') i=2;
	else if(Data[0] == 'd') i=3;
	else if(Data[0] == 'f') i=4;
	else if(Data[0] == 'p') i=5;
	else if(Data[0] == 'w') i=6;
	else if(Data[0] == 'R') i=7;
	else if(Data[0] == 'a') i=8;
	else if(Data[0] == 'm') i=9;

	switch(i)
	{
	case 1:
		Nextion_Change_Screen(5);
		current_page= PAGE_5;
		details_type=VEHICLE_COMPANY;
		break;

	case 2:
		Nextion_Change_Screen(5);
		current_page= PAGE_5;
		details_type=ENGINEER_NAME;
		break;

	case 3:
		Nextion_Change_Screen(5);
		current_page= PAGE_5;
		details_type=MODEL_NAME;
		break;

	case 4:
		Nextion_Change_Screen(5);
		current_page= PAGE_5;
		details_type=CLIENT_NAME;
		break;

	case 5:
		Nextion_Change_Screen(5);
		current_page= PAGE_5;
		details_type=VEHICLE_REG_NO;
		break;

	case 6:
		//print
		print_data();
		pendrive_save_data();
		break;

	case 7:
		//zero
		//zero_data();
		break;

	case 8:
		//save
		//save_data();
		break;

	case 9:
		//settingMenu();
		current_page = PAGE_2;

		break;

	}
}
void load_data()
{
	Nextion_Change_Screen(1);
	HAL_Delay(100);
	Flash_Read_Data(0x08007C00 ,(uint32_t *)vahicle_company, 5); //0x0801FC00
	Flash_Read_Data(0x08007800 ,(uint32_t *)engineer_name, 5);
	Flash_Read_Data(0x08007400 ,(uint32_t *)model_name, 5);
	Flash_Read_Data(0x08007000 ,(uint32_t *)client_name, 5);
	Flash_Read_Data(0x08006C00 ,(uint32_t *)vahicle_no, 5);

	Nextion_Send_Data("t28",vahicle_company);
	Nextion_Send_Data("t29",model_name);
	Nextion_Send_Data("t30",engineer_name);
	Nextion_Send_Data("t31",client_name);
	Nextion_Send_Data("t32",vahicle_no);
	//vahicle_company=(char)Rx_Data[0]/(float)100;
	//	setpoint=(float)Rx_Data[1]/(float)100;
	//	timer=Rx_Data[2];
}

void save_data()
{
	HAL_Delay(100);

	Flash_Write_Data(0x08007C00, (uint32_t *)vahicle_company,5); // 0x0801FC00 0x08006070 FLASH_ADDRESS
	Flash_Write_Data(0x08007800, (uint32_t *)engineer_name,5);
	Flash_Write_Data(0x08007400, (uint32_t *)model_name,5);
	Flash_Write_Data(0x08007000, (uint32_t *)client_name,5);
	Flash_Write_Data(0x08006C00, (uint32_t *)vahicle_no,5);

}

void load_data_display()
{
	Nextion_Send_Data("t28",vahicle_company);
	Nextion_Send_Data("t29",model_name);
	Nextion_Send_Data("t30",engineer_name);
	Nextion_Send_Data("t31",client_name);
	Nextion_Send_Data("t32",vahicle_no);
}

void Get_User_Data(char *data,uint8_t type)
{
	Nextion_Change_Screen(1);
	switch(type)
	{
	case VEHICLE_COMPANY:
		memset(vahicle_company,0,sizeof(vahicle_company));
		memcpy(vahicle_company,data,sizeof(vahicle_company));
		Nextion_Send_Data("t28",vahicle_company);

		break;

	case ENGINEER_NAME:
		memset(engineer_name,0,sizeof(engineer_name));
		memcpy(engineer_name,data,sizeof(engineer_name));
		Nextion_Send_Data("t30",engineer_name);
		break;

	case MODEL_NAME:
		memset(model_name,0,sizeof(model_name));
		memcpy(model_name,data,sizeof(model_name));
		Nextion_Send_Data("t29",model_name);
		break;

	case CLIENT_NAME:
		memset(client_name,0,sizeof(client_name));
		memcpy(client_name,data,sizeof(client_name));
		Nextion_Send_Data("t31",client_name);
		break;

	case VEHICLE_REG_NO:
		memset(vahicle_no,0,sizeof(vahicle_no));
		memcpy(vahicle_no,data,sizeof(vahicle_no));
		Nextion_Send_Data("t32",vahicle_no);
		break;

		//		case VEHICLE_COMPANY:
		//					break;
	}
	current_page = PAGE_1;
	data_update_flag=1;
	//Nextion_Change_Screen(1);
	save_data();
	load_data_display();
}
void setting_display(char *data)
{
	int i=0;
	if(data[0] == 's') i=1;
	else if(data[0] == 'A') i=2;
	else if(data[0] == 'c') i=3;
	else if(data[0] == 'k') i=4;
	//else if(Data[0] == 'd') i=4;
	switch(i)
	{
	case 1:
		current_page = PAGE_8;
		Nextion_Change_Screen(8);
		break;

	case 2:
		current_page = PAGE_4;
		//Nextion_Change_Screen(8);
		break;

	case 3:
		//count to display in setting menu
		break;

	case 4:
		current_page = PAGE_1;
		data_update_flag=1;
		//Nextion_Change_Screen(1);
		break;

	case 5:
		break;

	case 6:
		break;
	}
}
void calibration(char *data)
{
	int i=0;
	if(data[0] == 'k') i=5;
	else if(data[0] == 'z') i=6;
	else
	{
		i=data[0];
		i=49-i;
	}
	switch(i)
	{
	case 0:
		current_page = PAGE_9;
		Nextion_Change_Screen(9);
		calibration1(CMD_TCA9548A_CH0);
		break;

	case 1:
		current_page = PAGE_9;
		Nextion_Change_Screen(9);
		break;

	case 2:
		current_page = PAGE_9;
		Nextion_Change_Screen(9);
		break;

	case 3:
		current_page = PAGE_9;
		Nextion_Change_Screen(9);
		break;

	case 4:
		break;

	case 5:
		current_page = PAGE_1;
		data_update_flag=1;
		break;

	case 6:
		current_page = PAGE_2;
		break;
	}
}

void parameterData(char *data)
{
	int i=0;
	if(data[0] == 'k') i=5;
	else if(data[0] == 'z') i=6;
	switch(i)
	{
	case 5:
		current_page = PAGE_1;
		data_update_flag=1;
		break;

	case 6:
		current_page = PAGE_2;
		break;
	}
}
void NextionDisplay()
{
	switch(current_page)
	{
	case PAGE_1:
		if(data_update_flag != 0)
		{
			load_data_display();
			data_update_flag=0;
		}
		if(Data_Received_Ok == SET)
		{
			Edit_Details(rxData); //user data
			//current_page=5;
			Data_Received_Ok = RESET;
		}
		break;

	case PAGE_2:
		if(Data_Received_Ok == SET)
		{
			setting_display(rxData); //setting display
			Data_Received_Ok = RESET;
		}
		break;

	case PAGE_4:
		if(Data_Received_Ok == SET)
		{
			calibration(rxData); //calib data
			Data_Received_Ok = RESET;
		}
		break;
	case PAGE_5:
		if(Data_Received_Ok == SET)
		{
			Get_User_Data(rxData,details_type); //user data
			Data_Received_Ok = RESET;

		}
		break;

	case PAGE_8:
		if(Data_Received_Ok == SET)
		{
			parameterData(rxData); //parameter data
			Data_Received_Ok = RESET;

		}
		break;
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
	MX_I2C1_Init();
	MX_USART1_UART_Init();
	MX_USART3_UART_Init();
	MX_USART2_UART_Init();
	/* USER CODE BEGIN 2 */
	HAL_UART_Receive_IT(&huart1, (uint8_t*)usartRx, 1);

	load_data();

	I2C_channel_initADC(CMD_TCA9548A_CH0);
	//I2C_channel_initADC(CMD_TCA9548A_CH1);
	if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9)!=0)
	{
		//while (1)
		{
			//			BaseCount.value=read7802(CMD_TCA9548A_CH0);
			//			Uart_PutNumber(&huart1, BaseCount.value);
			//calibration1(CMD_TCA9548A_CH0);
		}

	}
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

		//HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		calculation(CMD_TCA9548A_CH0);
		wt_f=wt_f;
		//Uart_PutNumber(&huart1, wt_f);
		//HAL_Delay(20);

		NextionDisplay();
		HAL_Delay(100);
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

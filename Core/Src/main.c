/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RESET GPIO_PIN_13
#define BOOT0 GPIO_PIN_15
#define BOOT1 GPIO_PIN_14
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
UART_HandleTypeDef huart1;
int initialize= 0x71u;
uint8_t UCack;
uint8_t positive_ack=0x79;
uint8_t negative_ack=0xF1;
uint8_t version;
uint8_t commands[11];
uint8_t option1;
uint8_t option2;
uint8_t id[2];
uint8_t pages[2]={0x00,0x01};
uint8_t number_of_pages=0x01;
uint32_t length=2000;//20 in decimal
//uint8_t firmware[2000];
uint8_t read_address=0xa0;
uint8_t write_address=0xa1;
uint8_t read;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void send(uint8_t address);
uint8_t recieve();
int start();
int getversion();
int get_version_and_read_protection_status();
int get_id();
int read_memory(uint32_t address, uint8_t number_of_bytes,uint16_t start );
uint8_t get_checksum(uint32_t address);
void send_address(uint32_t address);
int go(uint32_t address);
int write(uint32_t address,uint8_t length,uint16_t start);
int write_unprotected();
int erase(uint8_t num_of_pages);
void restart();
int write_algorithm(uint32_t length);
void firmware_update(uint32_t length);
int get_backup(uint32_t length);

void make_array();
int EEPROM_init();
int EEPROM_write(uint16_t address,uint8_t data);
uint8_t EEPROM_read(uint16_t address);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  /* USER CODE BEGIN 2 */
	//  HAL_UART_Transmit(&huart1,(uint8_t*)(0xF7), 1, 10);
	//  HAL_UART_Transmit(&huart1,(uint8_t*)(0xF7), 1, 10);
	//  HAL_Delay(100);
	//  HAL_UART_Receive(&huart1,&UCack, 1,10);
	//  if(UCack==0x79){
	//	  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	//
	//  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
//  	restart();
//	start();
//	go(0x08000004);
//	//erase(number_of_pages);
//	HAL_Delay(50);
//	//start();
//	restart();
//	start();
//	write(0x08000230,0xf);
  //restart();
//  HAL_Delay(100);
//  restart();
//  while(!start()){
//	  restart();
//  }
  HAL_Delay(100);
  while(!(EEPROM_init()));
//  for(uint16_t address=0x00;address<=0xff;address=address+1){
//	  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
//	  HAL_Delay(10);
//	  if(HAL_I2C_IsDeviceReady(&hi2c1,0xA0,1,5)==HAL_OK){
//		  while(1){
//		  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
//		  HAL_Delay(1000);
//		  }
//	  	}
//  }
//  EEPROM_write(0xff,0x34);
//  EEPROM_write(0x1ff,0xDD);
//  for(uint16_t address=0x00;address<=0x4ff;address=address+1){
//	  while(!(EEPROM_write(address,address%128)));
//	  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
//	  HAL_Delay(25);
//  }
//  read= EEPROM_read(0x03);
//  read= EEPROM_read(0xffff);
//  make_array();
//  restart();
//  start();
//  firmware_update(0x8000);

  //restart();


//  HAL_Delay(100);
//  firmware_update(0x400);
//  get_backup(0x400);
//  firmware_update(0x400);
//  get_backup(0x400);



  //firmware_update(0x50);
  //go(0x08000004);
	//  getversion();
	//  get_version_and_read_protection_status();
	//  get_id();
	//  read_memory(0x08000000, 0xff);
  	//firmware_update(0x8000);
    //while(!EEPROM_write(0x00,0x00));
   	//get_backup(0x8000);
   	firmware_update(0x8000);
	while (1)
	{
//		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
//		HAL_Delay(25);
//		get_backup(0x10);
//		for(uint16_t address=0x00;address<=0x8000;address=address+1){
//			send(EEPROM_read(address));
//		}
		//HAL_Delay(100);
//		firmware_update(0x400);

		//HAL_Delay(500);
//		firmware_update(0x400);

//		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		//HAL_Delay(500);
		//	 if((USART1->SR & USART_SR_RXNE)){
		//		 char data=USART1->DR;
		//		 USART1->DR=data;
		//		 while(!(USART1->SR & USART_SR_TC));
		//	 }
    /* USER CODE END WHILE */

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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  hi2c1.Init.ClockSpeed = 400000;
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
  huart1.Init.WordLength = UART_WORDLENGTH_9B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_EVEN;
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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB13 PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void send(uint8_t address){
	USART1->DR=address;
	while(!(USART1->SR & USART_SR_TC));

}
uint8_t recieve(){
	while(!(USART1->SR & USART_SR_RXNE));
	uint8_t data=USART1->DR;
	return data;
}
uint8_t get_checksum(uint32_t address){
	uint8_t byte_1 = (address & 0xff000000) >> 24;
	uint8_t byte_2 = (address & 0x00ff0000) >> 16;
	uint8_t byte_3 = (address & 0x0000ff00) >> 8;
	uint8_t byte_4 = (address & 0x000000ff);
	uint8_t check_sum= byte_1^byte_2^byte_3^byte_4;

	return check_sum;

}
void send_address(uint32_t address){
	uint8_t byte = (address & 0xff000000) >> 24;
	send(byte);
	byte = (address & 0x00ff0000) >> 16;
	send(byte);
	byte= (address & 0x0000ff00) >> 8;
	send(byte);
	byte = (address & 0x000000ff);
	send(byte);
}
int start(){
	send(0x7F);
	if(recieve()==positive_ack){
		return 1;
		//HAL_UART_Transmit(&huart1, (uint8_t*)("Boot loader mode"), 25, 100);
	}
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	HAL_Delay(10);
	return 0;
}
int getversion(){
	uint8_t state=0x00;
	uint8_t number_of_bytes;
	while(1){
		switch (state)
		{
		case 0x01:
			number_of_bytes=recieve();
			//duplicate=number_of_bytes;
			uint8_t counter=0x00;
			while(counter<=number_of_bytes){
				if(counter==0x00){
					version=recieve();
				}else{
					commands[counter]=recieve();
				}
				counter=counter+0x01;
			}
			state=0x02;
			;
		case 0x02:
			if(positive_ack==recieve()){
				return 1;
			}else{
				return 0;
			}
			;
		default:
			send(0x00);
			send(0xff);
			if(recieve()==positive_ack){
				state=0x01;
			}else{
				return 0;
			}
			;
		}
	}
}

int get_version_and_read_protection_status(){
	uint8_t state=0x00;
	while(1){
		switch (state)
		{
		case 0x01:
			version=recieve();
			option1=recieve();
			option2=recieve();
			if(positive_ack==recieve()){
				return 1;
			}else{
				return -1;
			}
			;
		default:
			send(0x01);
			send(0xfe);
			if(recieve()==positive_ack){
				state=0x01;
			}else{
				return -1;
			}
			;
		}
	}
}
int get_id(){
	uint8_t state=0x00;
	uint8_t number_of_bytes=0x00;
	while(1){
		switch (state)
		{
		case 0x01:
			number_of_bytes=recieve();
			//duplicate=number_of_bytes;
			uint8_t counter=0x00;
			while(counter<=number_of_bytes){
				id[counter]=recieve();
				counter=counter+0x01;
			}
			state=0x02;
			;
		case 0x02:
			if(positive_ack==recieve()){
				return 1;
			}else{
				return -1;
			}
			;
		default:
			send(0x02);
			send(0xfd);
			if(recieve()==positive_ack){
				state=0x01;
			}else{
				return -1;
			}
			;
		}
	}
}
int read_memory(uint32_t address, uint8_t number_of_bytes,uint16_t start){
	uint8_t state=0x00;
	uint16_t counter=0x00;
	uint8_t buffer[256];
	while(1){
		switch (state)
		{
		case 0x01:
			send_address(address);
			send(get_checksum(address));
			//duplicate=number_of_bytes;
			if(positive_ack==recieve()){
				state=0x02;
			}else {
				return 0;
			}
			;
		case 0x02:
			send(number_of_bytes);
			send(~(number_of_bytes));
			if(positive_ack==recieve()){
				while(number_of_bytes>=counter){
					buffer[counter]=recieve();
					counter=counter+0x01;
				}
				counter=0x00;
				while(number_of_bytes>=counter){
					EEPROM_write(start, buffer[counter]);
					counter=counter+0x01;
					start=start+0x01;
				}
				return 1;
			}else{
				return 0;
			}
			;
		case 0x03:
			while(number_of_bytes>=counter){
				recieve();
				counter=counter+0x01;
			}
			counter=0x00;
			while(number_of_bytes>=counter){
				EEPROM_write(start, buffer[counter]);
				counter=counter+0x01;
				start=start+0x01;
			}
			return 1;
			;
		default:
			send(0x11);
			send(0xEE);
			if(recieve()==positive_ack){
				state=0x01;
			}else{
				return -1;
			}
			;
		}
	}

}

int go(uint32_t address){
	uint8_t state=0x00;
	while(1){
		switch (state)
		{
		case 0x01:
			send_address(address-0x04);
			send(get_checksum(address-0x04));
			if(positive_ack==recieve()){
				return 1;
			}else{
				return -1;
			}
			;
		default:
			send(0x21);
			send(0xde);
			if(recieve()==positive_ack){
				state=0x01;
			}else{
				return -1;
			}
			;
		}
	}

}
int write(uint32_t address,uint8_t length,uint16_t start){
	uint8_t state=0x00;
	uint16_t counter=0x00;
	uint8_t check_sum;
	uint8_t temp;
	while(1){
		switch (state)
		{
		case 0x01:
			send_address(address);
			send(get_checksum(address));
			if(positive_ack==recieve()){
				state=0x02;
			}else{
				return -1;
			}
			;
		case 0x02:
			send(length);
			check_sum=length;
			while(length>=counter){
				temp=EEPROM_read(start);
				send(temp);
				check_sum=check_sum ^temp ;
				counter=counter+1;
				start=start+1;
			}
			send(check_sum);
			state=0x03;
			;
		case 0x03:
			if(recieve()==positive_ack){
				return 1;
			}else{
				return -1;
			}
			;
		default:
			send(0x31);
			send(0xce);
			if(recieve()==positive_ack){
				state=0x01;
			}else{
				return -1;
			}
			;
		}
	}

}
int write_unprotected(){
	uint8_t state=0x00;
	while(1){
		switch (state)
		{
		case 0x01:
			if(positive_ack==recieve()){
				return 1;
			}else{
				return 0;
			}
			;
		default:
			send(0x73);
			send(0x8c);
			if(recieve()==positive_ack){
				state=0x01;
			}else{
				return 0;
			}
			;
		}
	}
}
int erase(uint8_t num_of_pages){
	uint8_t state=0x00;
	uint8_t check_sum=num_of_pages;
	uint8_t counter=0x00;
	while(1){
		switch (state)
		{
		case 0x01:
			send(num_of_pages);
			while(counter<=num_of_pages){
				send(counter);
				check_sum=check_sum ^ counter;
				counter=counter+1;
			}
			send(check_sum);
			if(recieve()==positive_ack){
				return 1;
			}else{
				return 0;
			}
			;
		default:
			send(0x43);
			send(0xbc);
			if(recieve()==positive_ack){
				state=0x01;
			}else{
				return 0;
			}
			;
		}
	}
}
void restart(){
  	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
  	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
  	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14, GPIO_PIN_RESET);
  	HAL_Delay(10);

  	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
  	HAL_Delay(10);
}
void firmware_update(uint32_t length){
	uint8_t erase_pages=(length/1024);
	if (length%1024>0){
		erase_pages=erase_pages+0x01;
	}
	erase_pages=erase_pages-0x01;

	//// erase the pages
	restart();

	HAL_Delay(100);

	while(!start()){
		restart();
	}

	HAL_Delay(100);
	while(!erase(erase_pages)){
		restart();
		while(!start());
	}
	//// erase the pages
	HAL_Delay(100);
	/// write the pages
	//restart();

	//HAL_Delay(10);

	while(!write_algorithm(length)){
		restart();
		while(!start());
	}

	/// write the pages

	go(0x8000004);
}

int  get_backup(uint32_t length){
	int read_times=(length/256);
	uint16_t start_1=0;
	uint32_t address=0x08000000;
	int status;
	if (length%256>0){
		read_times=read_times+0x01;
	}
	read_times=read_times-0x01;

	restart();
	HAL_Delay(100);

	while(!start()){
			restart();
		}
	HAL_Delay(100);

	while(read_times>=0){
		if(read_times==0){
			status=read_memory(address,length-1,start_1);
			if(status==0){
				return 0;
			}
		}else{
			status=read_memory(address,0xff,start_1);
			if(status==0){
				return 0;
			}
			start_1=start_1+0x100;
			length=length-0x100;
			address=address+0x100;
		}
		read_times=read_times-0x01;

	}

	return 1;
	//// erase the pages

}

int write_algorithm(uint32_t length){
	int count=length/256;
	int status;
	if(length%256>0){
		count=count+1;
	}
	uint16_t start=0;
	uint32_t address=0x08000000;
	while(count>0){
		if(count==1){
			status=write(address,length-1,start);
			if(!status){
				return 0;
			}
		}else{
			status=write(address,0xff,start);
			if(!status){
				return 0;
			}
			start=start+0x100;
			length=length-0x100;
			address=address+0x100;
			HAL_Delay(20); // there should be a delay
		}
		count=count-1;
		}
	return 1;
}
void make_array(){
	for(int i=0;i<2000;i++){
//		firmware[i]=i%128;
	}
}
int EEPROM_init(){
	if(HAL_I2C_IsDeviceReady(&hi2c1,0xA0,1,5)==HAL_OK){
		return 1;
	}else{
		return 0;
	}
}
int EEPROM_write(uint16_t address,uint8_t data){
	uint8_t address_data[3]={(address & 0xff00) >> 8,address & 0x00ff,data};
	uint8_t correct;

	HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)0xA0,address_data, 3, 100);// write to
	HAL_Delay(5);

//	correct=EEPROM_read(address);
//	if(correct==data){
//		return 1;
//	}

	return 1;
}

uint8_t EEPROM_read(uint16_t address){
	uint8_t _address[2]={(address & 0xff00) >> 8,address & 0x00ff};
	uint8_t read;

	HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)0xA0,_address,2,100);
	HAL_I2C_Master_Receive(&hi2c1,(uint16_t)0xA0, &read,1, 100);

//	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
//	HAL_Delay(10);

	return read;

}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

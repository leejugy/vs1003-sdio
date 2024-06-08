/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "dma.h"
#include "fatfs.h"
#include "sdio.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum{
	SD_READ=0,
	SD_WRITE,
	SD_NEW_WRITE
}
SD_RW_MODE;

typedef struct{
	uint8_t file_number;
	uint8_t file_name[256 + 3];
}sd_file_info;


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define	XCS_OFF HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET)
#define	XRST_OFF HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET)
#define	XDCS_OFF HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET)

#define	XCS_ON HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET)
#define	XRST_ON HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET)
#define	XDCS_ON HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET)

#define DREQ_VAL HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_1)

#define READ_VS1003_REG 0b11
#define WRITE_VS1003_REG 0b10

#define VS1003_MODE_REG 0x00
#define VS1003_CLOCKF_REG 0x03
#define VS1003_AUDATA_REG 0x05
#define VS1003_VOL_REG 0x0b
#define VS1003_DECODE_TIME 0x04

#define mp3_64kbps_stereo_sampling_rate 22000
#define mp3_64kbps_mono_sampling_rate 24000

/*
 * pb0 xcs
 * pb1 dreq
 * pa1 xrst
 * pa0 xdcs
 * */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t uart_rx_data;
uint8_t sd_buf[32];
uint16_t current_vol = 0x4141;

uint8_t file_name[259];

bool loop_flag = true;
bool stop_mp3 = false;

uint8_t sd_file_max_num;
sd_file_info sd_file[256];
uint8_t current_sd_file_num;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

//vs1003_control
uint16_t SCI_Read(uint8_t add);
void SCI_Write(uint8_t add,uint16_t write_val);
void vs1003_init();
void decoder_control();

//sd_control
void sd_user(SD_RW_MODE SD_RW,char file_derectory[]);
void SD_file_info();

//uart_control
void uart_send(uint8_t uart_num,char *fmt,...);

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance==USART1){
		HAL_UART_Receive_DMA(&huart1, &uart_rx_data, 1);
		if(uart_rx_data=='r'||
			 uart_rx_data=='1'||
			 uart_rx_data=='2'||
			 uart_rx_data=='3'||
			 uart_rx_data=='n'){

		if(uart_rx_data=='1' && strcmp((char *)file_name,"0:/a.mp3")==0){
			uart_send(1,"already in this song!.\n",current_vol);
			return;
		}
		else if(uart_rx_data=='2' && strcmp((char *)file_name,"0:/b.mp3")==0){
			uart_send(1,"already in this song!.\n",current_vol);
			return;
		}
		else if(uart_rx_data=='3' && strcmp((char *)file_name,"0:/c.mp3")==0){
			uart_send(1,"already in this song!.\n",current_vol);
			return;
		}
			loop_flag = false;
		}
		else if(uart_rx_data == 'i'){
			uart_rx_data = 0;
			for(uint8_t i=1;i<=sd_file_max_num;i++){
				uart_send(1,"%d.%s\n",i,sd_file[i].file_name);
			}
		}
		else if(uart_rx_data == ' '){
			stop_mp3 = (stop_mp3+1)%2;
			if(stop_mp3){
				uart_send(1,"stop.\n",current_vol);
			}
			else{
				uart_send(1,"resume.\n",current_vol);
			}
		}
		else if(uart_rx_data == '+'){
			if(current_vol-0x0101>=0){
				current_vol-=0x0101;
				uart_send(1,"%d%%vol_plus\n",(current_vol*100)/0xFEFE);
			}
			SCI_Write(VS1003_VOL_REG,current_vol);
		}
		else if(uart_rx_data == '-'){
			if(current_vol+0x0101<=0xfefe){
				current_vol+=0x0101;
				uart_send(1,"%d%%vol_minus\n",(current_vol*100)/0xFEFE);
			}
			SCI_Write(VS1003_VOL_REG,current_vol);
		}
	}
  UNUSED(huart);
}

void uart_send(uint8_t uart_num,char *fmt,...){
	uint8_t uart_send_data[48];
	memset(uart_send_data,0,48);
	va_list arg;
	va_start(arg,fmt);
	vsnprintf((char *)uart_send_data,48,fmt,arg);
	if(uart_num==1){
		HAL_UART_Transmit(&huart1, uart_send_data, 48, 10);
	}
	va_end(arg);
}

uint16_t SCI_Read(uint8_t add){
	uint16_t ret;
	uint8_t tx_data;
	uint8_t rx_data;
	while (DREQ_VAL == GPIO_PIN_RESET);

	XCS_ON;
	XDCS_OFF;

	tx_data = READ_VS1003_REG;
	HAL_SPI_Transmit(&hspi1, &tx_data, 1, 10);

	tx_data = add;
	HAL_SPI_Transmit(&hspi1, &tx_data, 1, 10);

	HAL_SPI_Receive(&hspi1, &rx_data, 1, 10);
	ret = rx_data<<8;

	HAL_SPI_Receive(&hspi1, &rx_data, 1, 10);
	ret |= rx_data;

	XCS_OFF;

	return ret;
}

void SCI_Write(uint8_t add,uint16_t write_val){
	uint8_t tx_data;
	while (DREQ_VAL == GPIO_PIN_RESET);

	XCS_ON;
	XDCS_OFF;

	tx_data = WRITE_VS1003_REG;
	HAL_SPI_Transmit(&hspi1, &tx_data, 1, 10);

	tx_data = add;
	HAL_SPI_Transmit(&hspi1, &tx_data, 1, 10);

	tx_data = write_val>>8;
	HAL_SPI_Transmit(&hspi1, &tx_data, 1, 10);

	tx_data = write_val;
	HAL_SPI_Transmit(&hspi1, &tx_data, 1, 10);

	XCS_OFF;

}

void SDI_Send(uint8_t *MP3_Data,size_t MP3_Len){
	XDCS_ON;
	for(uint8_t i=0;i<MP3_Len;i++){
		HAL_SPI_Transmit(&hspi1,&MP3_Data[i],1,10);
	}
	XDCS_OFF;
}

void vs1003_init(){
	XRST_ON;
	HAL_Delay(1);
	XRST_OFF;
	XCS_OFF;
	XDCS_OFF;
	uint16_t _16_data;
	uint8_t retry = 0;
  while(1){
  	_16_data = SCI_Read(VS1003_MODE_REG);
  	SCI_Write(VS1003_MODE_REG,0x0804);
  	if(_16_data==0x800 || retry++>=100){
  		uart_send(1,"ADD_MODE : 0x%04X\n",_16_data);
  		break;
  	}
  }

  _16_data = 0;
  retry = 0;
  while(1){
    _16_data = SCI_Read(VS1003_CLOCKF_REG);
    SCI_Write(VS1003_CLOCKF_REG,0x9BE8);
    if(_16_data==0x9BE8 || retry++>=100){
    	uart_send(1,"ADD_CLOCKF : 0x%04X\n",_16_data);
    	break;
   	}
  }

  _16_data = 0;
  retry = 0;
  while(1){
    _16_data = SCI_Read(VS1003_AUDATA_REG);
    SCI_Write(VS1003_AUDATA_REG,mp3_64kbps_stereo_sampling_rate);
    if(_16_data==mp3_64kbps_stereo_sampling_rate || retry++>=100){
    	uart_send(1,"ADD_AUDATA : 0x%04X\n",_16_data);
    	break;
   	}
  }

  _16_data = 0;
  retry = 0;
  while(1){
    _16_data = SCI_Read(VS1003_VOL_REG);
    SCI_Write(VS1003_VOL_REG,current_vol);
    if(_16_data==current_vol || retry++>=100){
    	uart_send(1,"ADD_VOL : 0x%04X\n",_16_data);
    	break;
   	}
  }

  _16_data = 0;
  retry = 0;
  while(1){
    _16_data = SCI_Read(VS1003_DECODE_TIME);
    SCI_Write(VS1003_DECODE_TIME,0x0000);
    if(_16_data==0x0000 || retry++>=100){
    	uart_send(1,"ADD_DECODE_TIME : 0x%04X\n",_16_data);
    	break;
   	}
  }
}

void SD_file_info(){
	DIR dp;
	FILINFO fno;
	bool do_flag = false;
	uint8_t i = 0;
	sd_file_max_num = 0;
	current_sd_file_num = 0;

	if(f_opendir(&dp, "0:/")==FR_OK){
		do_flag = true;
	}
	while(do_flag){
		f_readdir(&dp, &fno);
		if(dp.sect==0){
			uart_send(1,"complete\n");
			sd_file_max_num = i-1;
			break;
		}
		else{
			sd_file[i].file_number = i;
			sprintf((char *)sd_file[i].file_name,"0:/%s",(char *)fno.fname);
			i++;
		}
	}
}
void decoder_control(){
	if(uart_rx_data=='r'||
		 uart_rx_data=='1'||
		 uart_rx_data=='2'||
		 uart_rx_data=='3'||
		 uart_rx_data=='n'){
		switch (uart_rx_data){
		case '1':
			current_sd_file_num = 1;
		  sprintf((char *)file_name, "0:/a.mp3");
		  break;
		case '2':
			current_sd_file_num = 2;
		  sprintf((char *)file_name, "0:/b.mp3");
		  break;
		case '3':
			current_sd_file_num = 3;
		  sprintf((char *)file_name, "0:/c.mp3");
		  break;
		case 'n':
			if(current_sd_file_num++>=sd_file_max_num){
				current_sd_file_num = 1;
			}
			sprintf((char *)file_name, "%s",(char *)sd_file[current_sd_file_num].file_name);
			break;
		}
		uart_send(1,"you are in %d.\n",current_sd_file_num);
		loop_flag=true;
		uart_rx_data = 0;
		sd_user(SD_READ,(char *)file_name);
	}
}

void sd_user(SD_RW_MODE SD_RW,char file_derectory[]){
	UINT byte;
	uint32_t total_byte = 0;
  if(SD_RW==SD_READ){
  	if(f_open(&SDFile,(const TCHAR*)file_derectory,FA_OPEN_EXISTING|FA_READ)==FR_OK){
  		while(loop_flag){
  			f_read(&SDFile, sd_buf, 32, &byte);
  			total_byte+=byte;
  			while(DREQ_VAL==GPIO_PIN_RESET);
  			SDI_Send(sd_buf,32);
  			while(stop_mp3);
  			if(byte < 32){
  				loop_flag=false;
  			}
  		}
  	}
  	else{
  		uart_send(1,"error\n");
  	}
  }
  else if(SD_RW==SD_WRITE){
  	if(f_open(&SDFile,(const TCHAR*)file_derectory,FA_OPEN_EXISTING|FA_WRITE)==FR_OK){
    	f_read(&SDFile, sd_buf, 32, &byte);
    	uart_send(1,"\n%d byte read!\n",byte);
    }
    else{
    	uart_send(1,"error\n");
    }
  }
  else if(SD_RW==SD_NEW_WRITE){
    if(f_open(&SDFile,(const TCHAR*)file_derectory,FA_CREATE_NEW|FA_WRITE)==FR_OK){
     	f_read(&SDFile, sd_buf, 32, &byte);
     	uart_send(1,"\n%d byte read!\n",byte);
    }
     else{
     	uart_send(1,"error\n");
    }
  }
  uart_send(1,"total_byte : %d\n",total_byte);
  f_close(&SDFile);
}

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
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_SDIO_SD_Init();
  MX_FATFS_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  vs1003_init();
  UART_Start_Receive_DMA(&huart1, &uart_rx_data, 1);
  retSD = f_mount(&SDFatFS,(TCHAR const*)&SDPath[0],1);
  if(retSD==FR_OK){
    	uart_send(1,"f_mount ture!\n");
    }
  else{
    uart_send(1,"mount false error code : %d\n",retSD);
  }
  SD_file_info();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		decoder_control();
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 5;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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

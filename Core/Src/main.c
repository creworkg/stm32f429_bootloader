/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "usart.h"
#include "gpio.h"
#include "stdio.h"
#include "string.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

#define BYTE2WORD(highByte, lowByte) ((highByte<<8) | lowByte)
#define BYTE2DWORD(highByte, mhighByte, mlowByte, lowByte) \
        (highByte << 24) | (mhighByte << 16) | (mlowByte << 8) | (lowByte)

#define APP_START_ADDR 0x08008000

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
#define UPDATE_CMD "you need update"
#define UPDATE_CONFIRM "Oh,really?"
#define UPDATE_CHECKOUT "Oh,yes"
#define UPDATE_FINAL_CONFIRM "OK,come on"
#define RX_BUFF_SZ 255

uint8_t * updata_protocol = {
  UPDATE_CMD,
  UPDATE_CONFIRM,
  UPDATE_CHECKOUT,
  UPDATE_FINAL_CONFIRM
};
#define PROTOCOL_ITEM_NUM 4
#define FRAME_HEAD 0xA5
#define FRAME_TAIL 0xAA
#define FRAME_HEAD_LEN 1
#define FRAME_TAIL_LEN 1
#define FRAME_TAIL_LEN 1
#define FRAME_TAIL_LEN 1
#define FRAME_DATA_LEN 1
#define FRAME_ID_0  0x00 //frame info
#define FRAME_ID_1  0x01 //kerenl ID
#define FRAME_ID_2 0x02 //total data len
#define FRAME_ID_3 0x03 //
#define FRAME_ID_LEN 0x01
/*
frame info:
  frame head | frame info | frame pure data len | data | frame tail
  0xa5       | FRAME_ID   |     0~0xff          |  xxx |    0xaa
first frame:
total data len | frame len 

frist frame includes total data length
second frame includes pure data len in one frame
*/
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

HAL_StatusTypeDef flash_program(int startAddr, uint8_t * data, int dataSize);
HAL_StatusTypeDef flash_erase(int eraseStartAddr, int areaSize);
HAL_StatusTypeDef flash_lock();
HAL_StatusTypeDef flass_unlock();

HAL_StatusTypeDef flash_program(int startAddr, unsigned char* data, int dataSize)
{
  int start_addr = startAddr;
  int end_addr = start_addr + dataSize;
  int current_addr = start_addr;
  int i = 0;
  while (current_addr < end_addr){

    HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, current_addr, data[i]);
    current_addr++;
    i++;

  }

  return HAL_OK;

}
typedef struct{
  uint8_t frameHead;
  uint8_t frameID;
  uint8_t dataLen;
  uint8_t * dataHead;
  
} FRAME_INFO_ST, *FRAME_INFO_ST_P;

FRAME_INFO_ST frameInfo;

typedef enum UPDATE_STA{
  UPDATE_PRE,
  UPDATING,
  UPDATE_OK,
  UPDATE_FAIL
} E_UPDATE_STA;
enum FRAME_PARSE_RET{
  FRAME_OK,
  FRAME_TL,
  FRAME_KRNL,
  FRAME_DL_ERR,
  FRAME_HT_ERR,
  FRAME_OTHER_ERR

};
uint32_t totalDataLen;
uint8_t pureDataLen;
E_UPDATE_STA updateStatus;
static int frame_parse(uint8_t * buff, FRAME_INFO_ST_P frame_info)
{
  uint8_t * p = buff;
  uint8_t dataLen = p[2];
  if (dataLen < 0 || dataLen > 255)
  {
    /* code */
    return -1;
  }
  
  if (p[0] != FRAME_HEAD || p[dataLen] != FRAME_TAIL){
    return -2;
  }

  if (p[1] == FRAME_ID_0)
  {
    /* code */
    frame_info->frameID = FRAME_ID_0;
    frame_info->dataHead = &p[3];
    frame_info->dataLen = p[2];
    return FRAME_OK;
  }
  else if (p[1] == FRAME_ID_1)
  {
    /* code */
    frame_info->frameID = FRAME_ID_1;
    frame_info->dataHead = &p[3];
    frame_info->dataLen = p[2];
    return FRAME_KRNL;
  }
  else if (p[1] == FRAME_ID_2)
  {
    /* code */
    frame_info->frameID = FRAME_ID_2;
    frame_info->dataHead = &p[3];
    frame_info->dataLen = p[2];
    
    return FRAME_TL;
  }
  else
  {
    frame_info->frameID = 0xff;
    frame_info->dataHead = 0xff;
    frame_info->dataLen = 0xff;
    return FRAME_OTHER_ERR;
  }
  


  

}

#define SECTOR_0_FLASH_START_ADDR 0X08000000
#define SECTOR_1_FLASH_START_ADDR 0x08004000
#define SECTOR_2_FLASH_START_ADDR 0X08008000
#define SECTOR_3_FLASH_START_ADDR 0x0800C000
#define SECTOR_4_FLASH_START_ADDR 0X08010000
#define SECTOR_5_FLASH_START_ADDR 0x08020000
#define SECTOR_6_FLASH_START_ADDR 0X08040000
#define SECTOR_7_FLASH_START_ADDR 0x08060000
#define SECTOR_8_FLASH_START_ADDR 0X08080000
#define SECTOR_9_FLASH_START_ADDR 0x080A0000
#define SECTOR_10_FLASH_START_ADDR 0X080C0000
#define SECTOR_11_FLASH_START_ADDR 0x080E0000

static int start_sector(int eraseStartAddr)
{
  if (eraseStartAddr >= SECTOR_0_FLASH_START_ADDR && eraseStartAddr < SECTOR_1_FLASH_START_ADDR){
    return FLASH_SECTOR_0;
  }
  else if (eraseStartAddr >= SECTOR_1_FLASH_START_ADDR && eraseStartAddr < SECTOR_2_FLASH_START_ADDR) { 
    return FLASH_SECTOR_1;
  }
  else if (eraseStartAddr >= SECTOR_2_FLASH_START_ADDR && eraseStartAddr < SECTOR_3_FLASH_START_ADDR) { 
    return FLASH_SECTOR_2;
  }
  else if (eraseStartAddr >= SECTOR_3_FLASH_START_ADDR && eraseStartAddr < SECTOR_4_FLASH_START_ADDR) { 
    return FLASH_SECTOR_3;
  }
  else if (eraseStartAddr >= SECTOR_4_FLASH_START_ADDR && eraseStartAddr < SECTOR_5_FLASH_START_ADDR) { 
    return FLASH_SECTOR_4;
  }
  else if (eraseStartAddr >= SECTOR_5_FLASH_START_ADDR && eraseStartAddr < SECTOR_6_FLASH_START_ADDR) { 
    return FLASH_SECTOR_5;
  }
  else if (eraseStartAddr >= SECTOR_6_FLASH_START_ADDR && eraseStartAddr < SECTOR_7_FLASH_START_ADDR) { 
    return FLASH_SECTOR_6;
  }
  else if (eraseStartAddr >= SECTOR_7_FLASH_START_ADDR && eraseStartAddr < SECTOR_8_FLASH_START_ADDR) { 
    return FLASH_SECTOR_7;
  }
  else if (eraseStartAddr >= SECTOR_8_FLASH_START_ADDR && eraseStartAddr < SECTOR_9_FLASH_START_ADDR) { 
    return FLASH_SECTOR_8;
  }
  else if (eraseStartAddr >= SECTOR_9_FLASH_START_ADDR && eraseStartAddr < SECTOR_10_FLASH_START_ADDR) { 
    return FLASH_SECTOR_9;
  }
  else if (eraseStartAddr >= SECTOR_10_FLASH_START_ADDR && eraseStartAddr < SECTOR_11_FLASH_START_ADDR) { 
    return FLASH_SECTOR_10;
  }
  else if (eraseStartAddr >= SECTOR_11_FLASH_START_ADDR && eraseStartAddr < SECTOR_11_FLASH_START_ADDR+ 0x20000 ) { 
    return FLASH_SECTOR_11;
  }
  else{
    return 0xff;//addr error
  }
  
}
static int end_sector(int eraseStartAddr, int areaSize)
{
  return start_sector(eraseStartAddr + areaSize-1);
}

HAL_StatusTypeDef flash_erase(int eraseStartAddr, int areaSize)
{
  uint32_t SectorError = 0;
  FLASH_EraseInitTypeDef EraseInit;
  EraseInit.TypeErase = FLASH_TYPEERASE_SECTORS;
  EraseInit.Sector = start_sector(eraseStartAddr);
  EraseInit.NbSectors = end_sector(eraseStartAddr, areaSize) - EraseInit.Sector + 1;
  EraseInit.VoltageRange = FLASH_VOLTAGE_RANGE_3;

  return HAL_FLASHEx_Erase(&EraseInit, &SectorError);

}

HAL_StatusTypeDef flash_lock()
{
  return HAL_FLASH_Lock();
}

HAL_StatusTypeDef flash_unlock()
{
  return HAL_FLASH_Unlock();
}


typedef void(* pFunc)(void);
pFunc jumpToKernel;

void jump_to_kernel(uint32_t app_addr)
{
  //
  jumpToKernel = *(uint32_t *)(app_addr + 4);
  jumpToKernel();

}

void err_handle(void){
  HAL_UART_Transmit(&huart1, (uint8_t *)"handshake fail", 14, 1000);
  //jump to kerl
  jump_to_kernel(APP_START_ADDR);
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
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(GPIOH, GPIO_PIN_11, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOH, GPIO_PIN_12, GPIO_PIN_SET);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint8_t * pData = NULL;
  #define BUFF_SZ (RX_BUFF_SZ + FRAME_HEAD_LEN + FRAME_ID_LEN + FRAME_DATA_LEN + FRAME_TAIL_LEN)
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint8_t rx_buff[BUFF_SZ];
    u_int8_t frame_sz = 0;

    
    HAL_GPIO_TogglePin(GPIOH, GPIO_PIN_10);
    HAL_Delay(1000);
    HAL_UART_Transmit(&huart1, (uint8_t *)"hello world!", 12, 1000);

    //handshake
    for (int8_t i = 0; i < PROTOCOL_ITEM_NUM/2; i++)
    {
      /* code */
      HAL_UART_Receive(&huart1, rx_buff, 100, 5000);

      if(0 == memcmp(rx_buff, updata_protocol[2*i],strlen((const char*)updata_protocol[i])))
        HAL_UART_Transmit(&huart1, (uint8_t *)updata_protocol[2*i+1], strlen((const char*)updata_protocol[2*i+1]), 1000);
      else
        err_handle();
    }
    //get total data len 
    memset(rx_buff, 0, BUFF_SZ);
    HAL_UART_Receive(&huart1, rx_buff, 20, 1000);
    if(FRAME_TL == frame_parse(rx_buff, &frameInfo))
    {
      pData = frameInfo.dataHead;
      totalDataLen = BYTE2DWORD(pData[0], pData[1], pData[2], pData[3]);

      updateStatus = UPDATE_PRE;
    }

    //get frame pure data len 
    // memset(rx_buff, 0);
    // HAL_UART_Receive(&huart1, rx_buff, 20, 1000);
    // if (frame_sz > 0 && frame_sz <=256)
    // {
    //   /* code */
    //   HAL_UART_Transmit(&huart1, (uint8_t *)"copy that", 9, 1000);
    // }
    // else
    //   {
    //     HAL_UART_Transmit(&huart1, (uint8_t *)"frame size error", 16, 1000);
    //     goto ERR;
    //   }
    updateStatus = UPDATING;
    flash_unlock();
    //erase flash
    HAL_UART_Transmit(&huart1, (uint8_t *)"eraseing flash...", 17, 1000);
    flash_erase(APP_START_ADDR, totalDataLen);
    HAL_UART_Transmit(&huart1, (uint8_t *)"erase done", 10, 1000);

    //program flash after receive data
    uint32_t flash_program_addr = APP_START_ADDR;
    while (totalDataLen>0)
    {
      /* code */
      memset(rx_buff, 0, BUFF_SZ);
      HAL_UART_Receive(&huart1, rx_buff, BUFF_SZ, 5000);
      if(FRAME_KRNL ==  frame_parse(rx_buff, &frameInfo))
      {
        pData = frameInfo.dataHead;
        pureDataLen = frameInfo.dataLen;
        flash_program(flash_program_addr, pData, pureDataLen);
        flash_program_addr += pureDataLen;
        totalDataLen -= pureDataLen;
        HAL_UART_Transmit(&huart1, (uint8_t *)"next", 4, 1000);
      }

    }
    flash_lock();
    updateStatus = UPDATE_OK;
    HAL_UART_Transmit(&huart1, (uint8_t *)"program done", 12, 1000);
    //jump to kerl
    jump_to_kernel(APP_START_ADDR);

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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

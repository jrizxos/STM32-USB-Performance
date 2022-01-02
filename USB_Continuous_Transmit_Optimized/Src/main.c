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
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "usb_device.h"
#include "gpio.h"
#include "usbd_cdc_if.h"                  // added for CDC_Transmit_FS

/* Private define ------------------------------------------------------------*/
#define MSG_LEN     1024u                 // test message buffer length 
#define BUFF_LEN    50000u                // message lenght limit (50000 max tested)
#define PACKET_LEN  1024u                 // packet of data size

/* Private variables ---------------------------------------------------------*/
uint8_t send_mode = 1;                // when send_mode==true send data, set this to 1 if you want the program to send data immediatelly
                                      // otherwise 0 to wait until usr button press
uint8_t in_buff[MSG_LEN];             // we take data from this buffer pretending it was received by some IO
uint8_t usb_buff[2][BUFF_LEN];        // buffer to store a messages to be sent by USB

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Other_Init();
int COBS_msg(uint8_t * in_buff, const int start, uint16_t len, uint8_t * targ_buff, const int idx);

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void){
  uint8_t busy, buff_select = 0u;
  int i, in_idx = 0; 
  uint16_t buff_idx = 0u;

  /* MCU Configuration -------------------------------------------------------*/
  SCB_EnableICache();
  SCB_EnableDCache();
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_USB_DEVICE_Init();
  MX_DMA_Init();
  Other_Init();

  /* Prepare test data to send -----------------------------------------------*/
  // this is an emulation for data received by some IO periferal that needs to be sent to PC via USB
  // fills buffer with bytes from 0 to 255, such that: buffer[i] >= buffer[i-1]
  for(i=0 ; i<MSG_LEN ; i++){
    in_buff[i] = (uint8_t)(i/(uint8_t)((MSG_LEN/256u)+1));
  }

  /* Wait until first user button press --------------------------------------*/
  while (!send_mode) __NOP();

  /* Infinite loop -----------------------------------------------------------*/
  while (1){
    /* Get message from IO and write it to usb buffer */
    i = COBS_msg(in_buff, in_idx, PACKET_LEN, usb_buff[buff_select], buff_idx);
    in_idx += PACKET_LEN;
    if(in_idx+PACKET_LEN>MSG_LEN) in_idx = 0u;
    buff_idx += i;

    /* Try to transmit buffered data */
    if(buff_idx + PACKET_LEN <= BUFF_LEN){        // more packets fit in buffer
      busy = CDC_Transmit_FS(usb_buff[buff_select], buff_idx); // attempt to transmit
    }
    else{                                         // no more packets fit in buffer
      do{                                         // attempt to transmit
        busy = CDC_Transmit_FS(usb_buff[buff_select], buff_idx);
      }while(busy);                               // until usb is no longer busy
    }

    if(!busy){                                    // if data started to transmit
      buff_idx=0u;                                // reset buffer write idx
      if(++buff_select>=2) buff_select = 0u;  // select the next buffer to write in
    }
  }
}

/**
  * @brief Write len bytes of the message in the in_buff to targ_buff with Consistent Overhead Byte Stuffing
  * read starting from read_idx, write starting from buff_idx
  * @param in_buff: input buffer
  * @param read_idx: start reading from this index
  * @param len: length of bytes to copy
  * @param targ_buff: output buffer
  * @param buff_idx: start writing from this index
  * @retval Number of bytes written to targ_buff
  */
int COBS_msg(uint8_t * in_buff, const int read_idx, uint16_t len, uint8_t * targ_buff, const int buff_idx){
    int read_index = read_idx;
    int write_index = buff_idx;
    int code_index = write_index++;
    uint8_t code = 1;

    while(read_index < read_idx+len){
        if(in_buff[read_index] == 0){
            targ_buff[code_index] = code;
            code = 1;
            code_index = write_index++;
            read_index++;
        }
        else{
            targ_buff[write_index++] = in_buff[read_index++];
            code++;
            if(code == 0xFF){
                targ_buff[code_index] = code;
                code = 1;
                code_index = write_index++;
            }
        }
    }
    targ_buff[code_index] = code;
    targ_buff[write_index++] = 0u;

    return write_index-buff_idx;
}

/**
  * @brief Initiates user button, it's interrupt and one led (for nucleo FZ767ZI)
  * @retval None
  */
void Other_Init(){
  GPIO_InitTypeDef GPIO_InitStruct;
  // Configure GPIO pin : PC13 (user button)
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  // Configure GPIO pin : PB7 (blue led)
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  // Configure GPIO pin : PB14 (red led)
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  // Enable user button iterrupt
  HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_1);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
}

/**
  * @brief Button interrupt handler (called automatically)
  * @retval None
  */
void EXTI15_10_IRQHandler(void) {
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_13);
}

/**
  * @brief Button interrupt handler callback (called automatically), toggles bink variable and blue led
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  if(GPIO_Pin == GPIO_PIN_13){
    send_mode = 1u - send_mode;
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_7);
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_CLK48;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48SOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

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
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
    HAL_Delay(1000u);
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

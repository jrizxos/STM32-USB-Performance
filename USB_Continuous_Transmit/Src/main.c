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
#include "usbd_cdc_if.h"       // added for CDC_Transmit_FS

/* Private define ------------------------------------------------------------*/
#define MSG_LEN   1023    // message buffer length (50000 max tested)
#define BUFF_LEN  2u      // 2, changing this makes no difference

/* Private variables ---------------------------------------------------------*/
uint8_t send_mode = 1u;                   // when send_mode==true send data
uint8_t msg[MSG_LEN];                 // buffer to store and process the message before sending
uint8_t circ_buff[BUFF_LEN][MSG_LEN]; // circular buffer to store a messages while being sent by USB

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Other_Init();
void write_to_buff(uint8_t * msg, uint32_t len, uint8_t targ_buff[BUFF_LEN][MSG_LEN], uint8_t idx);
void XORed_msg(uint8_t * msg, uint32_t len, uint8_t targ_buff[BUFF_LEN][MSG_LEN], uint8_t idx, uint8_t * modif);
void COBS_msg(uint8_t * msg, uint32_t len, uint8_t targ_buff[BUFF_LEN][MSG_LEN], uint8_t idx);
void write_dma(uint32_t msg, uint32_t len, uint32_t targ_buff);
void rand_delay(uint8_t * state);
void LFSR(uint8_t * msg);

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void){
  uint8_t busy, buff_idx=0u;
  uint32_t i;

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();
  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();
  /* MCU Configuration--------------------------------------------------------*/
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_USB_DEVICE_Init();
  MX_DMA_Init();
  Other_Init();

  // Set initial state for lfsr
  uint8_t lfsr = 42u;
  // Prepare bytes of data to send
  // fills buffer with bytes from 0 to 255, such that: buffer[i] >= buffer[i-1]
  for(i=0 ; i<MSG_LEN-1 ; i++){
    msg[i] = (uint8_t)(i/(uint32_t)((MSG_LEN/256u)+1));
  }
  // sets last byte of msg to be the lfsr
  msg[MSG_LEN-1] = lfsr;
  /* Infinite loop */
  while (1){
    __NOP();                                          // loop gets stuck without this
    if(send_mode){                                    // if user button has been toggled on
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);     // start out pin processing msg signal

      //write_dma((uint32_t) &msg, MSG_LEN, (uint32_t) &circ_buff[buff_idx]);
      write_to_buff(msg, MSG_LEN, circ_buff, buff_idx);     // write to circular buffer
      XORed_msg(msg, MSG_LEN, circ_buff, buff_idx,  &lfsr);   // alter the message and write to circular buffer
      //COBS_msg(msg, (uint16_t)(254*(MSG_LEN-1)/255), circ_buff, buff_idx);
      //rand_delay(&lfsr);

      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);   // stop out pin processing msg signal
      do{                                         // attempt to send the message
        busy = CDC_Transmit_FS(circ_buff[buff_idx], MSG_LEN);
      }while(busy);                               // until usb is no longer busy
      
      if(buff_idx==BUFF_LEN-1) buff_idx = 0u;     // advance buffer index cyclically
      else buff_idx++;
      
    }
  }
}

/**
  * @brief Write msg using DMA and wait until it's done
  * @retval None
  */
void write_dma(uint32_t msg, uint32_t len, uint32_t targ_buff){
  extern uint8_t dma_done;  
  dma_done = 0u;
  HAL_DMA_Start_IT(&hdma_memtomem_dma2_stream0, msg, targ_buff, MSG_LEN); // use DMA to write message to circular buffer

  while(!dma_done){
    __NOP();
  }

}

/**
  * @brief Write msg, with length len, to position idx of buff
  * @retval None
  */
void write_to_buff(uint8_t *msg, uint32_t len, uint8_t targ_buff[BUFF_LEN][MSG_LEN], uint8_t idx){
  int i;
  for(i=0 ; i<len ; i++){    
    targ_buff[idx][i] = msg[i];
  }
}

/**
  * @brief Advance the LFSR state by one step
  * @retval None
  */
void LFSR(uint8_t * state){
  uint8_t lfsr = *state;
  uint8_t bit = ((lfsr >> 0) ^ (lfsr >> 2) ^ (lfsr >> 3) ^ (lfsr >> 4)) & 1u; // Taps: 8,6,5,4
  *state = (lfsr >> 1) | (bit << 7);
}

/**
  * @brief Xor every byte of the message with one modifier byte and write it to cicrular buffer
  * @retval None
  */
void XORed_msg(uint8_t * msg, uint32_t len, uint8_t targ_buff[BUFF_LEN][MSG_LEN], uint8_t idx, uint8_t *modif){
  int i;
  for(i=0 ; i<len-1 ; i++){                     // xor with modif and copy every byte from msg to targ_buff
    targ_buff[idx][i] = msg[i] ^ *modif;
    msg[i] = targ_buff[idx][i];
  }
  targ_buff[idx][len-1] = *modif;               // set last byte of both to modif
  msg[len-1] = *modif;
  LFSR(modif);                                  // update lfsr state (modif)
}

/**
  * @brief Perform Consistent Overhead Byte Stuffing to the msg while writing it to targ_buff
  *  !! provide len of data bytes to read from message such that [len + overhead stuffed bytes] <= MSG_LEN !!
  * @retval None
  */
void COBS_msg(uint8_t * msg, uint32_t len, uint8_t targ_buff[BUFF_LEN][MSG_LEN], uint8_t idx){

  uint8_t *encode = targ_buff[idx]; // Encoded byte pointer
	uint8_t *codep = encode++; // Output code pointer
	uint8_t code = 1; // Code value

	for (const uint8_t *byte = (const uint8_t *)msg; len--; ++byte)
	{
		if (*byte) // Byte not zero, write it
			*encode++ = *byte, ++code;

		if (!*byte || code == 0xff) // Input is zero or block completed, restart
		{
			*codep = code, code = 1, codep = encode;
			if (!*byte || len)
				++encode;
		}
	}
	*codep = code; // Write final code value
}

/**
  * @brief Wait for a random ammount of milliseconds
  * @retval None
  */
void rand_delay(uint8_t * state){
  HAL_Delay((*state)&3u);  // wait for 0 to 7 ms
  LFSR(state);            // update lfsr state
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

  // Configure GPIO pin : PA5 (out pin for cpu time)
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // Configure GPIO pin : PA4 (out pin for usb time)
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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

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

/* Private variables ---------------------------------------------------------*/
uint8_t send_mode = 1;                // when send_mode==true send data, set this to 1 if you want the program to send data immediatelly
                                      // otherwise 0 to wait until usr button press
uint8_t msg_buff[MSG_LEN];            // buffer to store and process the message before sending
uint8_t double_buff[2][MSG_LEN];      // circular buffer to store a messages while being sent by USB

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Other_Init();
void write_to_buff(uint8_t *msg_buff, uint32_t len, uint8_t * targ_buff);
void XORed_msg(uint8_t * msg_buff, uint32_t len, uint8_t * targ_buff, uint8_t *modif);
int COBS_msg(uint8_t * in_buff, const int start, uint16_t len, uint8_t * targ_buff, const int idx);
void write_dma(uint32_t msg_buff, uint32_t len, uint32_t targ_buff);
void rand_delay(uint8_t * state);
void LFSR(uint8_t * state);

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void){
  uint8_t busy, buff_select=0u;
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
  // Prepare some bytes of data to send
  // fills buffer with bytes from 0 to 255, such that: buffer[i] >= buffer[i-1]
  for(i=0 ; i<MSG_LEN-1 ; i++){
    msg_buff[i] = (uint8_t)(i/(uint32_t)((MSG_LEN/256u)+1));
  }
  // sets last byte of msg_buff to be the lfsr
  msg_buff[MSG_LEN-1] = lfsr;

  /* Infinite loop */
  while (1){
    __NOP();                                          // loop gets stuck without a nop, due to a compiler optimization bug
    if(send_mode){                                    // if user button has been pressed once
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);     // set output pin that signals message processing

      // select only one workload and uncomment
      //write_dma((uint32_t) &msg_buff, MSG_LEN, (uint32_t) &double_buff[buff_select]);
      //write_to_buff(msg_buff, MSG_LEN, double_buff[buff_select]);
      XORed_msg(msg_buff, MSG_LEN, double_buff[buff_select], &lfsr);
      //COBS_msg(msg_buff, 0, MSG_LEN, double_buff[buff_select], 0);

      // uncomment to add some extra random delay 
      //rand_delay(&lfsr);

      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);   // reset output pin that signals message processing
      do{                                         // attempt to send the message
        busy = CDC_Transmit_FS(double_buff[buff_select], MSG_LEN);
      }while(busy);                               // until usb is no longer busy
      
      if(++buff_select>=2) buff_select = 0u;  // select the next buffer to write in
    }
  }
}

/**
  * @brief Write msg_buff using DMA and wait until it's done
  * @param  msg_buff: input buffer
  * @param  len: length of bytes to copy
  * @param  targ_buff: output buffer 
  * @retval None
  */
void write_dma(uint32_t msg_buff, uint32_t len, uint32_t targ_buff){
  extern uint8_t dma_done;  // gets set in dma.c
  dma_done = 0u;
  HAL_DMA_Start_IT(&hdma_memtomem_dma2_stream0, msg_buff, targ_buff, MSG_LEN); // use DMA to write message to circular buffer

  while(!dma_done){ // wait util DMA is done, the message is not ready to be sent
    __NOP();        // here you could do something useful in a real aplication
  }

}

/**
  * @brief Copies len bytes from input buffer to output buffer
  * @param  msg_buff: input buffer
  * @param  len: length of bytes to copy
  * @param  targ_buff: output buffer
  * @retval None
  */
void write_to_buff(uint8_t *msg_buff, uint32_t len, uint8_t * targ_buff){
  int i;
  for(i=0 ; i<len ; i++){    
    targ_buff[i] = msg_buff[i];
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
  * @param  msg_buff: input buffer
  * @param  len: length of bytes to copy
  * @param  targ_buff: output buffer
  * @param  modif: every copied byte gets XORed with this modifier
  * @retval None
  */
void XORed_msg(uint8_t * msg_buff, uint32_t len, uint8_t * targ_buff, uint8_t *modif){
  int i;
  for(i=0 ; i<len-1 ; i++){                     // xor with modif and copy every byte from msg_buff to targ_buff
    targ_buff[i] = msg_buff[i] ^ *modif;
    msg_buff[i] = targ_buff[i];
  }
  targ_buff[len-1] = *modif;               // set last byte of both to modif
  msg_buff[len-1] = *modif;
  LFSR(modif);                                  // update lfsr state (modif)
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
  * @brief Wait for a random ammount of milliseconds
  * @retval None
  */
void rand_delay(uint8_t * state){
  HAL_Delay((*state)&3u);  // wait for 0 to 7 ms
  LFSR(state);            // update lfsr state
}

/**
  * @brief Initiates user button, the button's interrupt and one led (for Nucleo FZ767ZI)
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
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_13); // toggle blue led
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

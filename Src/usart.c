/**
  ******************************************************************************
  * File Name          : USART.c
  * Description        : This file provides code for the configuration
  *                      of the USART instances.
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "usart.h"
#include "motor.h"
#include "gpio.h"
#include "dma.h"

/* USER CODE BEGIN 0 */

#include "cmsis_os.h"
#include "task.h"
#include <string.h>
#include "CRC.c"

uint8_t uartRxBuffer[RX_BUFFER_LENGTH];
static uint8_t uartRxProcessingBuffer[RX_BUFFER_LENGTH];

float motorPosCmd[5];

static uint8_t sofPos	= 0;
static uint8_t dataLength = 0;
static uint8_t frameSeq = 0;
uint8_t frameID = 0;

osThreadId uartRxThreadHandle;
osThreadId uartTxThreadHandle;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;


/* USER CODE END 0 */

/* USART1 init function */

void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 500000;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */
    /* USART1 clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();
  
    /**USART1 GPIO Configuration    
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX 
    */
    GPIO_InitStruct.Pin = STM_TX_Pin|STM_RX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART1 DMA Init */
    /* USART1_RX Init */
    hdma_usart1_rx.Instance = DMA1_Channel5;
    hdma_usart1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart1_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_rx.Init.Mode = DMA_NORMAL;
    hdma_usart1_rx.Init.Priority = DMA_PRIORITY_MEDIUM;
    if (HAL_DMA_Init(&hdma_usart1_rx) != HAL_OK)
    {
      _Error_Handler(__FILE__, __LINE__);
    }

    __HAL_LINKDMA(uartHandle,hdmarx,hdma_usart1_rx);

    /* USART1_TX Init */
    hdma_usart1_tx.Instance = DMA1_Channel4;
    hdma_usart1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_usart1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart1_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_tx.Init.Mode = DMA_NORMAL;
    hdma_usart1_tx.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_usart1_tx) != HAL_OK)
    {
      _Error_Handler(__FILE__, __LINE__);
    }

    __HAL_LINKDMA(uartHandle,hdmatx,hdma_usart1_tx);

    /* USART1 interrupt Init */
    HAL_NVIC_SetPriority(USART1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspInit 1 */

  /* USER CODE END USART1_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();
  
    /**USART1 GPIO Configuration    
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX 
    */
    HAL_GPIO_DeInit(GPIOA, STM_TX_Pin|STM_RX_Pin);

    /* USART1 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmarx);
    HAL_DMA_DeInit(uartHandle->hdmatx);

    /* USART1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
  }
} 

/* USER CODE BEGIN 1 */

/* UART line idle detected, DMA UART receive operation has been stoped, restart now */

/*
void  HAL_UART_AbortReceiveCpltCallback (UART_HandleTypeDef *huart) {
  //notify receiver thread there's new data
 // osSignalSet(uartRxThreadHandle, UART_DATA_AVAILABLE);
  //arm idle line interupt trigger
  //UART_DMA_STOP_AT_IDLE(*huart);
  //restart receiver
  HAL_UART_Receive_DMA(huart, uartRxBuffer, sizeof(uartRxBuffer));

}
*/

/* debug
int count =0;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  count++;
}
*/

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {

  memcpy(uartRxProcessingBuffer, uartRxBuffer, RX_BUFFER_LENGTH);

    
  for (sofPos = 0; sofPos < RX_BUFFER_LENGTH; sofPos++) {
    if (uartRxProcessingBuffer[sofPos] == START_OF_FRAME) {
      if (Verify_CRC8_Check_Sum(uartRxProcessingBuffer + sofPos, SIZE_FRAMEHEAD)) {
        if (Verify_CRC16_Check_Sum(uartRxProcessingBuffer + sofPos, SIZE_FRAMEHEAD + 
        SIZE_FRAMEID + SIZE_FRAMETAIL + (uartRxProcessingBuffer[OFFSET_DATA_LENGTH + sofPos] << 8 |
        uartRxProcessingBuffer[OFFSET_DATA_LENGTH + sofPos + 1]))) {

          dataLength = uartRxProcessingBuffer[OFFSET_DATA_LENGTH + sofPos] << 8 | 
                       uartRxProcessingBuffer[OFFSET_DATA_LENGTH + sofPos + 1];
       //   frameID = uartRxProcessingBuffer[OFFSET_DATA_TYPE + sofPos] << 8 | 
       //             uartRxProcessingBuffer[OFFSET_DATA_TYPE + sofPos + 1];
          frameSeq = uartRxProcessingBuffer[OFFSET_FRAME_SEQ];    

          memcpy(motorPosCmd, uartRxProcessingBuffer + sofPos + OFFSET_DATA_PAC, 20);
          break;
        }
      }
    }
  }
//  hdma_usart1_rx.State = HAL_DMA_STATE_READY;
}
/*
void uartSendBuffer(const float* data, uint16_t dataSize) {
  uint16_t dataType = 0;
  uint8_t txBuffer[64];
  static uint8_t txCount = 0;
  txCount++;
  memset(txBuffer, 0, sizeof(txBuffer));
  txBuffer[0] = START_OF_FRAME;
  txBuffer[1] = (uint8_t) dataSize >> 8;
  txBuffer[2] = (uint8_t) dataSize;
  txBuffer[OFFSET_FRAME_SEQ] = txCount;
  Append_CRC8_Check_Sum(txBuffer, SIZE_FRAMEHEAD);
  txBuffer[5] = (uint8_t) dataType >> 8;
  txBuffer[6] = (uint8_t) dataType;
  uint16_t currentSize = OFFSET_DATA_TYPE + sizeof(uint16_t); //7
  memcpy(txBuffer + currentSize, data, dataSize);
  currentSize += dataSize;
  Append_CRC16_Check_Sum(txBuffer, currentSize + SIZE_FRAMETAIL);
  currentSize += 2;
  HAL_UART_Transmit(&huart1, (uint8_t *) txBuffer, 64, 50); 
}
*/

void uartSendBuffer(const float* data, uint16_t dataSize) {

  uint8_t txBuffer[22];
  memset(txBuffer, 0, sizeof(txBuffer)); // fill with zeros
  txBuffer[0] = START_OF_FRAME;
  memcpy(txBuffer + 1, data, dataSize);
  txBuffer[21] = END_OF_FRAME; // append end of frame
  HAL_UART_Transmit(&huart1, (uint8_t *) txBuffer, 22, 50);  // 18 bytes. 100 ms delay
}


void debugPrintLn(UART_HandleTypeDef *huart, char _out[]){
       HAL_UART_Transmit_DMA(huart, (uint8_t *) _out, strlen(_out));
       char newline[2] = "\r\n";
       HAL_UART_Transmit_DMA(huart, (uint8_t *) newline, 2);
}

// uint8_t temp[5]; //debug
void uartRxThreadFunction(void const * argument) {

  while(1)
  {
  if (hdma_usart1_rx.State == HAL_DMA_STATE_READY) {
    HAL_UART_Receive_DMA(&huart1, (uint8_t *) &uartRxBuffer, RX_BUFFER_LENGTH);
  //  HAL_UART_Receive_DMA(&huart1, (uint8_t *) &temp, 5); // debug
  //  hdma_usart1_rx.State = HAL_DMA_STATE_BUSY;
  }
  osDelay(100);
  HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
  }
}

/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

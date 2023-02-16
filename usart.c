/**
  ******************************************************************************
  * File Name          : USART.c
  * Description        : This file provides code for the configuration
  *                      of the USART instances.
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

/* Includes ------------------------------------------------------------------*/
#include "usart.h"

/* USER CODE BEGIN 0 */
#include "radio/hdlc.h"
//#include "Qtest.h"
#ifdef __ELECSOFT_1939__
#include "common.h"
#else // __ELECSOFT_1939__
#include "Common.h"
#endif // __ELECSOFT_1939__
#include "string.h"
#include "ncp.h"
#include "global_vars.h"
#include "application.h"
#include "watchdog_task.h"

#include "timer.h"

static osThreadId vTaskUartRxHandle;
#ifdef __INNOVA_20220623__  // uart task size
static uint32_t vTaskUartRxBuffer[2048];
#else
static uint32_t vTaskUartRxBuffer[1400];
#endif						 // uart task size
static osStaticThreadDef_t vTaskUartRxControlBlock;

static SemaphoreHandle_t UART_TX_Acess = NULL;                         // Semaphore to protect access to buffer msg
static StaticSemaphore_t UART_TX_Acess_buffer;

static uint32_t modbus_tx_finish = 0;

extern osMessageQId uQueueISRUartReceiverHandle;                       // Queue to receive radio data from DMA
static uint8_t aRxBuffer[RXBUFFERSIZE];                                // Buffer to receive usart radio data by DMA

/* USER CODE END 0 */

UART_HandleTypeDef hlpuart1;
UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_lpuart_tx;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

/* LPUART1 init function */

void MX_LPUART1_UART_Init(void)
{

  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 115200;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_RTS_CTS;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }

}
/* USART1 init function */

void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
#ifdef __ELECSOFT_2074__
  huart1.Init.HwFlowCtl = UART_HWCONTROL_RTS_CTS;
#else
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;//UART_HWCONTROL_RTS_CTS;
#endif
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==LPUART1)
  {
  /* USER CODE BEGIN LPUART1_MspInit 0 */

  /* USER CODE END LPUART1_MspInit 0 */
    /* LPUART1 clock enable */
    __HAL_RCC_LPUART1_CLK_ENABLE();

    __HAL_RCC_GPIOC_CLK_ENABLE();
    /**LPUART1 GPIO Configuration
    PC0     ------> LPUART1_RX
    PC1     ------> LPUART1_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_LPUART1;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* LPUART1 DMA Init */
    /* LPUART_TX Init */
    hdma_lpuart_tx.Instance = DMA2_Channel6;
    hdma_lpuart_tx.Init.Request = DMA_REQUEST_4;
    hdma_lpuart_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_lpuart_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_lpuart_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_lpuart_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_lpuart_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_lpuart_tx.Init.Mode = DMA_NORMAL;
    hdma_lpuart_tx.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_lpuart_tx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmatx,hdma_lpuart_tx);

    /* LPUART1 interrupt Init */
    HAL_NVIC_SetPriority(LPUART1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(LPUART1_IRQn);
  /* USER CODE BEGIN LPUART1_MspInit 1 */
    HAL_NVIC_SetPriority(LPUART1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(LPUART1_IRQn);

  /* USER CODE END LPUART1_MspInit 1 */
  }
  else if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */
    /* USART1 clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    PA11     ------> USART1_CTS
    PA12     ------> USART1_RTS
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART1 DMA Init */
    /* USART1_RX Init */
    hdma_usart1_rx.Instance = DMA1_Channel5;
    hdma_usart1_rx.Init.Request = DMA_REQUEST_2;
    hdma_usart1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart1_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_rx.Init.Mode = DMA_CIRCULAR;
    hdma_usart1_rx.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_usart1_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmarx,hdma_usart1_rx);

    /* USART1_TX Init */
    hdma_usart1_tx.Instance = DMA1_Channel4;
    hdma_usart1_tx.Init.Request = DMA_REQUEST_2;
    hdma_usart1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_usart1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart1_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_tx.Init.Mode = DMA_NORMAL;
    hdma_usart1_tx.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_usart1_tx) != HAL_OK)
    {
      Error_Handler();
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

  if(uartHandle->Instance==LPUART1)
  {
  /* USER CODE BEGIN LPUART1_MspDeInit 0 */

  /* USER CODE END LPUART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_LPUART1_CLK_DISABLE();

    /**LPUART1 GPIO Configuration
    PC0     ------> LPUART1_RX
    PC1     ------> LPUART1_TX
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_0|GPIO_PIN_1);

    /* LPUART1 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmatx);

    /* LPUART1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(LPUART1_IRQn);
  /* USER CODE BEGIN LPUART1_MspDeInit 1 */
    HAL_NVIC_DisableIRQ(LPUART1_IRQn);

  /* USER CODE END LPUART1_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();

    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    PA11     ------> USART1_CTS
    PA12     ------> USART1_RTS
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12);

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

HAL_StatusTypeDef Modbus_Uart_Change_settings(uint32_t baudrate, uint32_t Stopbit, uint32_t wordlength, uint32_t parity)
{
    HAL_StatusTypeDef ret = HAL_BUSY;

    hlpuart1.Init.BaudRate = 19200;

    if((baudrate < 9600) || (baudrate > 5000000))
    {
        ret = HAL_ERROR;
    }
    if((Stopbit != UART_STOPBITS_0_5) && (Stopbit != UART_STOPBITS_1) &&
       (Stopbit != UART_STOPBITS_1_5) && (Stopbit != UART_STOPBITS_2))
    {
        ret = HAL_ERROR;
    }
    if((wordlength != UART_WORDLENGTH_7B) && (wordlength != UART_WORDLENGTH_8B) && (wordlength != UART_WORDLENGTH_9B))
    {
        ret = HAL_ERROR;
    }
    if((parity != UART_PARITY_NONE) && (parity != UART_PARITY_EVEN) && (parity != UART_PARITY_ODD))
    {
        ret = HAL_ERROR;
    }


    if(ret != HAL_ERROR)
    {
        hlpuart1.Init.BaudRate = baudrate;
        hlpuart1.Init.WordLength = wordlength;
        hlpuart1.Init.StopBits = Stopbit;
        hlpuart1.Init.Parity = parity;
        HAL_UART_Abort(&hlpuart1);
        HAL_UART_DeInit(&hlpuart1);

        if (HAL_UART_Init(&hlpuart1) != HAL_OK) {
            Error_Handler();
        }

        //MX_UART1_Start();
        //ret = UART_SetConfig(&hlpuart1);
    }

    return ret;
}

HAL_StatusTypeDef ModBus_Uart_Start() {

    hlpuart1.Instance->ICR = USART_ICR_TCCF ;

    __HAL_UART_ENABLE_IT(&hlpuart1, UART_IT_TC | UART_IT_RXNE);  // enable idle line interrupt

    return HAL_OK;

//    if(uQueueISRModbusHandle != NULL)
//    {
//        xQueueReset(uQueueISRModbusHandle);
//    }
//
//    if(modbus_tx_sem != NULL)
//    {
//        xQueueReset(modbus_tx_sem);
//
//    }
//    else
//    {
//        modbus_tx_sem = xSemaphoreCreateBinaryStatic( &modbus_tx_sem_buffer );
//    }
//
//    xSemaphoreGive(modbus_tx_sem); // Is a semaphore so allow to enter
//
//    return HAL_UART_Receive_DMA(&hlpuart1, (uint8_t*) modbusRxBuffer, sizeof(modbusRxBuffer));
}

HAL_StatusTypeDef MX_UART1_Start() {

    huart1.Instance->ICR = USART_ICR_IDLECF;

	__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);  // enable idle line interrupt]

	if(uQueueISRUartReceiverHandle != NULL)
	{
	    xQueueReset(uQueueISRUartReceiverHandle);
	}

	if(UART_TX_Acess != NULL)
	{
	    xQueueReset(UART_TX_Acess);
	    xSemaphoreGive(UART_TX_Acess); // Is a semaphore so allow to enter
	}

	return HAL_UART_Receive_DMA(&huart1, (uint8_t*) aRxBuffer, sizeof(aRxBuffer));

}

void MX_USART1_UART_Baudrate(uint32_t baudrate)
{
    HAL_UART_Abort(&huart1);
    HAL_UART_DeInit(&huart1);
    huart1.Init.BaudRate = baudrate;

    if (HAL_UART_Init(&huart1) != HAL_OK) {
        Error_Handler();
    }

    MX_UART1_Start();
}

void MX_USART1_UART_Restart(void)
{
    HAL_UART_Abort(&huart1);
    //HAL_UART_Init(&huart1);
    MX_UART1_Start();
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == LPUART1)
    {
        modbus_tx_finish = 0;
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {

    /* Set transmission flag: transfer complete */
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if(huart->Instance == USART1)
    {
        if(UART_TX_Acess != NULL)
        {
            xSemaphoreGiveFromISR(UART_TX_Acess, &xHigherPriorityTaskWoken); //Usart transmision complete
        }
    }
    else if(huart->Instance == LPUART1)
    {
        modbus_tx_finish = 0;
    }

    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}
//timer_st timer_radio;

/* USER CODE BEGIN Header_UartReceiver */
/**
 * @brief Function implementing the vTaskUartRx thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_UartReceiver */
void UartReceiverTask(void const *argument)
{
	/* USER CODE BEGIN UartReceiver */
	uint16_t pxRxedMessage = 0;
	uint16_t HeadRx = 0;
	radio_t *radio = get_radio();
	uint8_t buffer[HDLC_MSG_RX_BUFFER_SIZE];

	xQueueReset(uQueueISRUartReceiverHandle);

	// Use same buffer to decode slip and hdlc frame because never is going to be received at the same time
    initHDLCRx(&radio->msgRxHdlc, buffer, HDLC_MSG_RX_BUFFER_SIZE);
    slip_init(&radio->boot.slip, buffer, HDLC_MSG_RX_BUFFER_SIZE);
	memset(aRxBuffer, 0x00, sizeof(aRxBuffer));

	while(MX_UART1_Start() == HAL_ERROR)
	{
	    osDelay(100);
	}

	wdt_id wdt_id = -1;

	/* Infinite loop */
	for (;;)
	{
	    if(wdt_id < 0)
	    {
	        wdt_id = wdt_enable_thread();
	    }
	    else
	    {
	        wdt_set_thread(wdt_id);
	    }

	    if (xQueueReceive(uQueueISRUartReceiverHandle, &pxRxedMessage, (radio->status.state.radio_found == 0)?100:1000) == pdTRUE)
		{
	        while (HeadRx != pxRxedMessage)
			{
	            if(radio_process_byte(radio, aRxBuffer[HeadRx]) == 1)
				{
				    //start_timer(&timer_radio);
				    radio->status.state.radio_found = 1;
				}
	            if (++HeadRx >= RXBUFFERSIZE)
				{
					HeadRx = 0;
				}
			}
		}
	    else
	    {
	        if(radio->status.state.radio_found == 0)
	        {
	            if((huart1.Init.BaudRate == 1000000) && (radio->boot_active== 0))
	            {
	                MX_USART1_UART_Baudrate(115200);
	                radio->boot_active = 1;
	                nordic_dfu_check_bootloader(&radio->boot);
	            }
	            else
	            {
	                if(radio->boot_active == 1)
	                {
	                    MX_USART1_UART_Baudrate(115200);
	                    radio->boot_active = 0;
	                    resetHDLCRx(&radio->msgRxHdlc);
	                }
	                else
	                {
	                    MX_USART1_UART_Baudrate(1000000);
	                }
	            }

	        }
	    }
	}
	/* USER CODE END UartReceiver */
}

void Init_UartReceiverTask(osPriority priority)
{
#ifdef __INNOVA_20220623__  // uart task size
    osThreadStaticDef(vTaskUartRx, UartReceiverTask, priority, 0, 2048, vTaskUartRxBuffer, &vTaskUartRxControlBlock);//antes osPriorityLow
#else
    osThreadStaticDef(vTaskUartRx, UartReceiverTask, priority, 0, 1400, vTaskUartRxBuffer, &vTaskUartRxControlBlock);//antes osPriorityLow
#endif  					 // uart task size
    vTaskUartRxHandle = osThreadCreate(osThread(vTaskUartRx), NULL);
}

#ifdef __ELECSOFT_1931_cast__
HAL_StatusTypeDef Usart_boot_Transmit(uint8_t *pData, uint16_t nByte)
#else // __ELECSOFT_1931_cast__
HAL_StatusTypeDef Usart_boot_Transmit(uint8_t *pData, uint32_t nByte)
#endif // __ELECSOFT_1931_cast__
{
    static uint8_t msg[1300];           // Made static to save RAM
    uint8_t IDLE = TRUE;
    HAL_StatusTypeDef ret = HAL_BUSY;
    if(UART_TX_Acess == NULL)
    {
        // If semaphore is not create, create it
        UART_TX_Acess = xSemaphoreCreateBinaryStatic( &UART_TX_Acess_buffer );
        IDLE = TRUE;
    }
    else
    {
        // Time to wait (1 / UartSpeed) * MaxSize * 2 = (1 / 115200) * 1500 * 2 = 30 ms
        if(xSemaphoreTake(UART_TX_Acess, (30 / portTICK_PERIOD_MS) ) == pdFALSE )
        {
            // Cannot get semafore, we cannot tx the data
            IDLE = FALSE;
        }
    }

    if(IDLE == TRUE)
    {
        if((nByte * 2) < sizeof(msg))
        {
            uint32_t msg_size = 0;

            if(slip_encode( msg, pData, nByte, &msg_size) == SLIP_STATUS_SUCCESS)
            {
#ifdef __ELECSOFT_1931_cast__ // To be double checked
                // Bear in mind 'nByte' is 'uint16_t', 'msg_size' should not be greater than 'uint16_t'
                if(HAL_UART_Transmit_DMA(&huart1, msg, (uint16_t)msg_size) != HAL_OK)
#else // __ELECSOFT_1931_cast__
                if(HAL_UART_Transmit_DMA(&huart1, msg, msg_size) != HAL_OK)
#endif // __ELECSOFT_1931_cast__
                {
                    // If an error with Uart transmit free the semaphore and provide error
                    ret = HAL_ERROR;
                    xSemaphoreGive(UART_TX_Acess);
                }
                else
                {
                    ret = HAL_OK;
                }
            }
        }
    }

    return ret;
}

#ifdef __ELECSOFT_1931_cast__
HAL_StatusTypeDef Usart_radio_Transmit(uint8_t *pData, uint16_t nByte)
#else // __ELECSOFT_1931_cast__
HAL_StatusTypeDef Usart_radio_Transmit(uint8_t *pData, uint32_t nByte)
#endif // __ELECSOFT_1931_cast__
{
    static uint8_t msg[1300];           // Made static to save RAM
    uint8_t IDLE = TRUE;
    HAL_StatusTypeDef ret = HAL_BUSY;
    if(UART_TX_Acess == NULL)
    {
        // If semaphore is not create, create it
        UART_TX_Acess = xSemaphoreCreateBinaryStatic( &UART_TX_Acess_buffer );
        IDLE = TRUE;
    }
    else
    {
        // Time to wait (1 / UartSpeed) * MaxSize * 2 = (1 / 115200) * 1500 * 2 = 30 ms
        if(xSemaphoreTake(UART_TX_Acess, (100 / portTICK_PERIOD_MS) ) == pdFALSE )
        {
            // Cannot get semafore, we cannot tx the data
            IDLE = FALSE;
        }
    }

    if(IDLE == TRUE)
    {
#ifdef __ELECSOFT_1931_cast__
        uint16_t msg_size = hdlcEncoder( pData, nByte, msg, sizeof(msg));
#else // __ELECSOFT_1931_cast__
        uint32_t msg_size = hdlcEncoder( pData, nByte, msg, sizeof(msg));
#endif // __ELECSOFT_1931_cast__

        if(HAL_UART_Transmit_DMA(&huart1, msg, msg_size) != HAL_OK)
        {
            // If an error with Uart transmit free the semaphore and provide error
            ret = HAL_ERROR;
            xSemaphoreGive(UART_TX_Acess);
        }
        else
        {
            ret = HAL_OK;
        }
    }

    return ret;
}

#ifdef __ELECSOFT_1931_cast__
HAL_StatusTypeDef Usart_Transmit(uint8_t *pData, uint16_t nByte)
#else // __ELECSOFT_1931_cast__
HAL_StatusTypeDef Usart_Transmit(uint8_t *pData, uint32_t nByte)
#endif // __ELECSOFT_1931_cast__
{
    HAL_StatusTypeDef ret;
    radio_t *radio = get_radio();

    if(radio->boot_active == 0)
    {
        ret = Usart_radio_Transmit(pData, nByte);
    }
    else
    {
        ret = Usart_boot_Transmit(pData, nByte);
    }

    return ret;
}

uint32_t Modbus_receive(uint8_t *byte)
{
    uint32_t ret = 0;
    if((hlpuart1.Instance->ISR & USART_ISR_RXNE) != 0)
    {
        *byte = (uint8_t)(hlpuart1.Instance->RDR & 0xFF);
        ret = 1;
    }

    if((hlpuart1.Instance->ISR & (USART_ISR_PE | USART_ISR_FE | USART_ISR_NE | USART_ISR_ORE)) != 0)
    {
        hlpuart1.Instance->ICR = (USART_ICR_PECF | USART_ICR_FECF | USART_ICR_NECF | USART_ICR_ORECF);
    }

    return ret;
}

#ifdef __ELECSOFT_1931_cast__
uint32_t Modbus_Transmit(uint8_t *pData, uint16_t nByte)
#else // __ELECSOFT_1931_cast__
uint32_t Modbus_Transmit(uint8_t *pData, uint32_t nByte)
#endif // __ELECSOFT_1931_cast__
{
    uint32_t ret = 0;
    timer_st timer;

    hlpuart1.Instance->ICR = USART_ICR_TCCF;
    __HAL_UART_ENABLE_IT(&hlpuart1, UART_IT_TC);

    while(modbus_tx_finish == 1)
    {
        osDelay(1);
    }
    modbus_tx_finish = 1;

    if(HAL_UART_Transmit_DMA(&hlpuart1, pData, nByte) != HAL_OK)
    {
        modbus_tx_finish = 0;
        ret  = 0;
    }
    else
    {
        ret = 1;
    }
    start_timer(&timer);

    while((modbus_tx_finish == 1) && (elapsed_timer(&timer) < 1000))
    {
        osDelay(1);
    }

    return ret;
}





/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

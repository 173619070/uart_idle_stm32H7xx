/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "cmsis_os.h"
#include "crc.h"
#include "dma.h"
#include "fdcan.h"
#include "hash.h"
#include "quadspi.h"
#include "rng.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"
#include "lwrb.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;
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
static void MPU_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */


/* USART1 init function */
__ALIGNED(32) uint8_t dma_rx_buf[DMA_BUFFER_LENGTH] ;//__attribute__((section(".RAM_D2")));
__ALIGNED(32) uint8_t dma_tx_buf[DMA_BUFFER_LENGTH] ;//__attribute__((section(".RAM_D2"))));


/**
 * \brief           Ring buffer data array for RX DMA
 */
uint8_t usart_rx_rb_data[DMA_BUFFER_LENGTH];
volatile size_t usart_rx_dma_current_len = 0;
/**
 * \brief           Ring buffer data array for TX DMA
 */
uint8_t usart_tx_rb_data[DMA_BUFFER_LENGTH];


/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
volatile size_t usart_tx_dma_current_len;
/* USER CODE END 0 */


void EnableUSART_IDLE(void);


/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  //MPU_Config();

  /* Enable I-Cache---------------------------------------------------------*/
  //SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  //SCB_EnableDCache();

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

  MX_CRC_Init();
  MX_FDCAN1_Init();
  MX_HASH_Init();
  MX_RNG_Init();
  //MX_QUADSPI_Init();
		
	MX_USB_DEVICE_Init();
	
  MX_DMA_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  EnableUSART_IDLE();
  /* USER CODE END 2 */

	
  /* Call init function for freertos objects (in freertos.c) */
  //MX_FREERTOS_Init();
  /* Start scheduler */
  //osKernelStart();


	while(1) {
		usart_tx_thread(0);

	}
}



/**
 * \brief           Process received data over UART
 * \note            Either process them directly or copy to other bigger buffer
 * \param[in]       data: Data to process
 * \param[in]       len: Length in units of bytes
 */
void usart_process_data(const void* data, size_t len) {
	memcpy(usart_rx_rb_data, data, len);
	usart_rx_dma_current_len += len;
}

/**
 * \brief           Check for new data received with DMA
 *
 * User must select context to call this function from:
 * - Only interrupts (DMA HT, DMA TC, UART IDLE) with same preemption priority level
 * - Only thread context (outside interrupts)
 *
 * If called from both context-es, exclusive access protection must be implemented
 * This mode is not advised as it usually means architecture design problems
 *
 * When IDLE interrupt is not present, application must rely only on thread context,
 * by manually calling function as quickly as possible, to make sure
 * data are read from raw buffer and processed.
 *
 * Not doing reads fast enough may cause DMA to overflow unread received bytes,
 * hence application will lost useful data.
 *
 * Solutions to this are:
 * - Improve architecture design to achieve faster reads
 * - Increase raw buffer size and allow DMA to write more data before this function is called
 */
void usart_rx_check() 
{
    static size_t old_pos;

    /* Calculate current position in buffer and check for new data available */
    uint32_t pos = DMA_BUFFER_LENGTH - __HAL_DMA_GET_COUNTER(huart1.hdmarx);

    if (pos != old_pos) {                       /* Check change in received data */
        if (pos > old_pos) {                    /* Current position is over previous one */
            /*
             * Processing is done in "linear" mode.
             *
             * Application processing is fast with single data block,
             * length is simply calculated by subtracting pointers
             *
             * [   0   ]
             * [   1   ] <- old_pos |------------------------------------|
             * [   2   ]            |                                    |
             * [   3   ]            | Single block (len = pos - old_pos) |
             * [   4   ]            |                                    |
             * [   5   ]            |------------------------------------|
             * [   6   ] <- pos
             * [   7   ]
             * [ N - 1 ]
             */
            usart_process_data(&dma_rx_buf[old_pos], pos - old_pos);
        } else {
            /*
             * Processing is done in "overflow" mode..
             *
             * Application must process data twice,
             * since there are 2 linear memory blocks to handle
             *
             * [   0   ]            |---------------------------------|
             * [   1   ]            | Second block (len = pos)        |
             * [   2   ]            |---------------------------------|
             * [   3   ] <- pos
             * [   4   ] <- old_pos |---------------------------------|
             * [   5   ]            |                                 |
             * [   6   ]            | First block (len = N - old_pos) |
             * [   7   ]            |                                 |
             * [ N - 1 ]            |---------------------------------|
             */
            usart_process_data(&dma_rx_buf[old_pos], DMA_BUFFER_LENGTH - old_pos);
            if (pos > 0) {
                usart_process_data(&dma_rx_buf[0], pos);
            }
        }
        old_pos = pos;                          /* Save current position as old for next transfers */
    } 
		else 
		{
			osDelay(1);
		}
}


void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t Size)
{

		//本次串口接受32个字节容量
		//HAL_UART_Receive_DMA(&huart1,rx_buffer,32);//打开DMA接收
		//然后我发送了0x0C个字节的数据长度
		//那么此时__HAL_DMA_GET_COUNTER函数返回值就是0x14个字节，表示还能接收多少字节达到容量上限
		//_len_dmarev = 预先定义的接收总字节(0x20) - __HAL_DMA_GET_COUNTER()（0x14） = 0x0C
		//HAL_UART_DMAStop(huart);//Circular模式这里调用这个函数就失去了Circular模式的效果了
		//pos = DMA_BUFFER_LENGTH - __HAL_DMA_GET_COUNTER(huart1.hdmarx);

		SCB_InvalidateDCache_by_Addr((uint32_t*)dma_rx_buf, DMA_BUFFER_LENGTH);

		usart_rx_check();

		HAL_UARTEx_ReceiveToIdle_DMA(huart, dma_rx_buf, DMA_BUFFER_LENGTH);
	
}


void
usart_rx_dma_thread(void const* argument) 
{
    /* Notify user to start sending data */
    while (1) {
        /* Simply call processing function */
        usart_rx_check();
    }
}


/**
 * \brief           Check if DMA is active and if not try to send data
 *
 * This function can be called either by application to start data transfer
 * or from DMA TX interrupt after previous transfer just finished
 *
 * \return          `1` if transfer just started, `0` if on-going or no data to transmit
 */

void
usart_start_tx_dma_transfer(void) {

    /*
     * First check if transfer is currently in-active,
     * by examining the value of usart_tx_dma_current_len variable.
     *
     * This variable is set before DMA transfer is started and cleared in DMA TX complete interrupt.
     *
     * It is not necessary to disable the interrupts before checking the variable:
     *
     * When usart_tx_dma_current_len == 0
     *    - This function is called by either application or TX DMA interrupt
     *    - When called from interrupt, it was just reset before the call,
     *         indicating transfer just completed and ready for more
     *    - When called from an application, transfer was previously already in-active
     *         and immediate call from interrupt cannot happen at this moment
     *
     * When usart_tx_dma_current_len != 0
     *    - This function is called only by an application.
     *    - It will never be called from interrupt with usart_tx_dma_current_len != 0 condition
     *
     * Disabling interrupts before checking for next transfer is advised
     * only if multiple operating system threads can access to this function w/o
     * exclusive access protection (mutex) configured,
     * or if application calls this function from multiple interrupts.
     *
     * This example assumes worst use case scenario,
     * hence interrupts are disabled prior every check
     */


    uint32_t primask;
    uint8_t started = 0;

    primask = __get_PRIMASK();
    __disable_irq();

		if (usart_tx_dma_current_len == 0)
		{
			__HAL_DMA_DISABLE(&hdma_usart1_tx);

			  //LL_DMA_ClearFlag_TC1(DMA1);
        //LL_DMA_ClearFlag_HT1(DMA1);
        //LL_DMA_ClearFlag_TE1(DMA1);
        //LL_DMA_ClearFlag_DME1(DMA1);
        //LL_DMA_ClearFlag_FE1(DMA1);
			
        __HAL_DMA_CLEAR_FLAG(&hdma_usart1_tx, __HAL_DMA_GET_TC_FLAG_INDEX(&hdma_usart1_tx));
        __HAL_DMA_CLEAR_FLAG(&hdma_usart1_tx, __HAL_DMA_GET_HT_FLAG_INDEX(&hdma_usart1_tx));
        __HAL_DMA_CLEAR_FLAG(&hdma_usart1_tx, __HAL_DMA_GET_TE_FLAG_INDEX(&hdma_usart1_tx));
				//__HAL_DMA_CLEAR_FLAG(&hdma_usart1_tx, __HAL_DMA_GET_DME_FLAG_INDEX(&hdma_usart1_tx));
				//__HAL_DMA_CLEAR_FLAG(&hdma_usart1_tx, __HAL_DMA_GET_FE_FLAG_INDEX(&hdma_usart1_tx));
        //__HAL_DMA_CLEAR_FLAG(&hdma_usart1_tx, __HAL_DMA_GET_GI_FLAG_INDEX(&hdma_usart1_tx));

        HAL_UART_Transmit_DMA(&huart1, (uint8_t*)usart_tx_rb_data, usart_tx_dma_current_len);
			
			__HAL_DMA_ENABLE(&hdma_usart1_tx);

		}
		__set_PRIMASK(primask);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    usart_tx_dma_current_len = 0;           /* Clear length variable */
    usart_start_tx_dma_transfer();          /* Start sending more data */
}

	
void usart_tx_thread(void const* argument) 
{
    /* Notify user to start sending data */
    uint8_t state, cmd, len, size, idx, err;
    uint8_t pkg_data[64];
		idx = 0;
    err = 0;
    state = 0;
		len = 0;
    while (1)
    {
        uint8_t b;

        /* Process RX ringbuffer */

        /* Packet format: START_BYTE, CMD, LEN[, DATA[0], DATA[len - 1]], STOP BYTE */
        /* DATA bytes are included only if LEN > 0 */
        /* An example, send sequence of these bytes: 55 FF 02 11 22 AA */

        /* Read byte by byte */

        if (size > 0 && ((len == 0 && usart_rx_dma_current_len > 0)
					|| (len > 0 && size >= len + 1 && usart_rx_dma_current_len > 0)))
        {
            switch (state)
            {
            case 0:
            {           /* Wait for start byte */
                if (b == 0x55)
                {
                    idx = 0;
                    ++state;
                }
                else
                {
                    //包处理异常
                    err = 1;
                }
            }
            break;
            case 1:
            {           /* Check packet command */
                len = b;
                ++state;
            }
            break;
            case 2:
            {           /* Packet data length */
                cmd = b;
								if (len > 0)
								{
									--len;
								}
								else
								{
									err = 1;
								}
                ++state;
            }
            break;
            default:
            {         /* End of packet */
                if (b == 0xAA) {
                    /* Packet is valid */

                    state = 0;
                    idx = 0;
                }
                else
                {
                    if (len > 0)
                    {
                        --len;          /* Decrease for received character */
                        pkg_data[idx] = b;//len=[1,0]  data=[11,22],len=0
                        ++idx;//[1,2]
                        ++state;
                    }
                    else
                    {
                        //包处理异常
                        err = 1;
                    }
                }
                break;
            }
            }

            /* Do other tasks ... */
        }
				else 
				{
						osDelay(1);
				}
			}
}


void EnableUSART_IDLE(void)
{
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, dma_rx_buf, DMA_BUFFER_LENGTH);
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 12;
  RCC_OscInitStruct.PLL.PLLR = 8;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x24000000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_512KB;
  MPU_InitStruct.SubRegionDisable = 0x0;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL1;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
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

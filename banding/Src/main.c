/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */
/* memory and peripheral start addresses */
 
/* pin definitions */
#define Pin0 0
#define Pin1 1
#define Pin2 2
#define Pin3 3
#define Pin4 4
#define Pin5 5
#define Pin6 6
#define Pin7 7
#define Pin8 8

// #define OUTPUT_MODE     (0x10|0x03) // output mode: push-pull + 50MHz
 
/* RCC peripheral addresses applicable to GPIOA */
// #define RCC_APB2ENR     (*(volatile unsigned long*)(RCC_BASE + 0x18))
 
/* GPIOA peripheral addresses */
// #define GPIOA_CRL   (*(volatile unsigned long*)(GPIOA_BASE + 0x00))
// #define GPIOA_CRH   (*(volatile unsigned long*)(GPIOA_BASE + 0x04))

#define GPIOA_Hi    (*(volatile unsigned long*)(GPIOA_BASE + 0x10))
#define GPIOA_Lo    (*(volatile unsigned long*)(GPIOA_BASE + 0x14))

// #define GPIOB_CRL   (*(volatile unsigned long*)(GPIOB_BASE + 0x00))
// #define GPIOB_CRH   (*(volatile unsigned long*)(GPIOB_BASE + 0x04))

#define GPIOB_Hi    (*(volatile unsigned long*)(GPIOB_BASE + 0x10))
#define GPIOB_Lo    (*(volatile unsigned long*)(GPIOB_BASE + 0x14))
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
DMA_HandleTypeDef hdma_memtomem_dma1_channel1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
__STATIC_INLINE void DelayMicro(uint32_t __IO micros)
{
  micros *=(SystemCoreClock/1000000)/5;
  while(micros--);
}

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
/* vector table */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
    /* enable clock on GPIOA peripheral */
    //RCC_APB2ENR = 0x04;
     
    /* pin output mode */
//     GPIOA_CRL |= OUTPUT_MODE << ((P0) << 2);
//     GPIOA_CRH |= OUTPUT_MODE << ((P0) << 2);
//     GPIOA_CRL |= OUTPUT_MODE << ((P5) << 2);

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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

  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
   /*
     uint8_t j;
     for (j=0x01; j<=0x80; j<<=1) // Mask changed  from 0x01 (P1.1) up to 0x80 (P1.7)
     {
       if (j==0) j=0x01;
         GPIOA_Lo = j;
       HAL_Delay(3);
       GPIOA_Hi = j;
     }
     HAL_Delay(3);
   */
      float i;

     for(i=200;i<1500;i+=100)
     {
       GPIOA_Hi = 1<<Pin0;
       GPIOA_Lo = 1<<Pin1;
       GPIOA_Hi = 1<<Pin2;
       GPIOA_Lo = 1<<Pin3;
       GPIOA_Hi = 1<<Pin4;
       GPIOA_Lo = 1<<Pin5;
       GPIOA_Hi = 1<<Pin6;
       GPIOA_Lo = 1<<Pin7;
       GPIOA_Hi = 1<<Pin8;
       DelayMicro(i*100);
       GPIOA_Lo = 1<<Pin0;
       GPIOA_Hi = 1<<Pin1;
       GPIOA_Lo = 1<<Pin2;
       GPIOA_Hi = 1<<Pin3;
       GPIOA_Lo = 1<<Pin4;
       GPIOA_Hi = 1<<Pin5;
       GPIOA_Lo = 1<<Pin6;
       GPIOA_Hi = 1<<Pin7;
       GPIOA_Lo = 1<<Pin8;
       DelayMicro(i*100);
     }
     if(i>1200)
     for(i=1500;i>200;i-=100)
     {
       GPIOA_Hi = 1<<Pin0;
       GPIOA_Lo = 1<<Pin1;
       GPIOA_Hi = 1<<Pin2;
       GPIOA_Lo = 1<<Pin3;
       GPIOA_Hi = 1<<Pin4;
       GPIOA_Lo = 1<<Pin5;
       GPIOA_Hi = 1<<Pin6;
       GPIOA_Lo = 1<<Pin7;
       GPIOA_Hi = 1<<Pin8;
       DelayMicro(i*100);
       GPIOA_Lo = 1<<Pin0;
       GPIOA_Hi = 1<<Pin1;
       GPIOA_Lo = 1<<Pin2;
       GPIOA_Hi = 1<<Pin3;
       GPIOA_Lo = 1<<Pin4;
       GPIOA_Hi = 1<<Pin5;
       GPIOA_Lo = 1<<Pin6;
       GPIOA_Hi = 1<<Pin7;
       GPIOA_Lo = 1<<Pin8;
       DelayMicro(i*100);
     }

     GPIOA_Hi = 1<<Pin0;
     GPIOA_Hi = 1<<Pin1;
     GPIOA_Hi = 1<<Pin2;
     GPIOA_Hi = 1<<Pin3;
     GPIOA_Hi = 1<<Pin4;
     GPIOA_Hi = 1<<Pin5;
     GPIOA_Hi = 1<<Pin6;
     GPIOA_Hi = 1<<Pin7;
     GPIOA_Hi = 1<<Pin8;
     DelayMicro(500000);

     for(i=200;i<1500;i+=100)
     {
       GPIOB_Hi = 1<<Pin0;
       GPIOB_Lo = 1<<Pin1;
       GPIOB_Hi = 1<<Pin2;
       GPIOB_Lo = 1<<Pin3;
       GPIOB_Hi = 1<<Pin4;
       GPIOB_Lo = 1<<Pin5;
       GPIOB_Hi = 1<<Pin6;
       GPIOB_Lo = 1<<Pin7;
       GPIOB_Hi = 1<<Pin8;
       DelayMicro(i*100);
       GPIOB_Lo = 1<<Pin0;
       GPIOB_Hi = 1<<Pin1;
       GPIOB_Lo = 1<<Pin2;
       GPIOB_Hi = 1<<Pin3;
       GPIOB_Lo = 1<<Pin4;
       GPIOB_Hi = 1<<Pin5;
       GPIOB_Lo = 1<<Pin6;
       GPIOB_Hi = 1<<Pin7;
       GPIOB_Lo = 1<<Pin8;
       DelayMicro(i*100);
     }
     if(i>1200)
     for(i=1500;i>200;i-=100)
     {
       GPIOB_Hi = 1<<Pin0;
       GPIOB_Lo = 1<<Pin1;
       GPIOB_Hi = 1<<Pin2;
       GPIOB_Lo = 1<<Pin3;
       GPIOB_Hi = 1<<Pin4;
       GPIOB_Lo = 1<<Pin5;
       GPIOB_Hi = 1<<Pin6;
       GPIOB_Lo = 1<<Pin7;
       GPIOB_Hi = 1<<Pin8;
       DelayMicro(i*100);
       GPIOB_Lo = 1<<Pin0;
       GPIOB_Hi = 1<<Pin1;
       GPIOB_Lo = 1<<Pin2;
       GPIOB_Hi = 1<<Pin3;
       GPIOB_Lo = 1<<Pin4;
       GPIOB_Hi = 1<<Pin5;
       GPIOB_Lo = 1<<Pin6;
       GPIOB_Hi = 1<<Pin7;
       GPIOB_Lo = 1<<Pin8;
       DelayMicro(i*100);
     }

     GPIOB_Hi = 1<<Pin0;
     GPIOB_Hi = 1<<Pin1;
     GPIOB_Hi = 1<<Pin2;
     GPIOB_Hi = 1<<Pin3;
     GPIOB_Hi = 1<<Pin4;
     GPIOB_Hi = 1<<Pin5;
     GPIOB_Hi = 1<<Pin6;
     GPIOB_Hi = 1<<Pin7;
     GPIOB_Hi = 1<<Pin8;
     DelayMicro(500000);

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/** 
  * Enable DMA controller clock
  * Configure DMA for memory to memory transfers
  *   hdma_memtomem_dma1_channel1
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* Configure DMA request hdma_memtomem_dma1_channel1 on DMA1_Channel1 */
  hdma_memtomem_dma1_channel1.Instance = DMA1_Channel1;
  hdma_memtomem_dma1_channel1.Init.Direction = DMA_MEMORY_TO_MEMORY;
  hdma_memtomem_dma1_channel1.Init.PeriphInc = DMA_PINC_ENABLE;
  hdma_memtomem_dma1_channel1.Init.MemInc = DMA_MINC_ENABLE;
  hdma_memtomem_dma1_channel1.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma_memtomem_dma1_channel1.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
  hdma_memtomem_dma1_channel1.Init.Mode = DMA_NORMAL;
  hdma_memtomem_dma1_channel1.Init.Priority = DMA_PRIORITY_LOW;
  if (HAL_DMA_Init(&hdma_memtomem_dma1_channel1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3 
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_3|GPIO_PIN_4 
                          |GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA0 PA1 PA2 PA3 
                           PA4 PA5 PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3 
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB3 PB4 
                           PB5 PB6 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_3|GPIO_PIN_4 
                          |GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

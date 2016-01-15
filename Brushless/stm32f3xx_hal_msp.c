/**
  ******************************************************************************
  * File Name          : stm32f3xx_hal_msp.c
  * Date               : 27/04/2015 12:33:08
  * Description        : This file provides code for the MSP Initialization
  *                      and de-Initialization codes.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2015 STMicroelectronics
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
#include "stm32f3xx_hal.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * Initializes the Global MSP.
  */
void HAL_MspInit(void)
{
  /* USER CODE BEGIN MspInit 0 */

  /* USER CODE END MspInit 0 */

  HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  /* System interrupt init*/
/* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 2, 0);

  /* USER CODE BEGIN MspInit 1 */

  /* USER CODE END MspInit 1 */
}

void HAL_ADC_MspInit(ADC_HandleTypeDef* hadc)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(hadc->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspInit 0 */

  /* USER CODE END ADC1_MspInit 0 */
    /* Peripheral clock enable */
    __ADC1_CLK_ENABLE();

    /**ADC1 GPIO Configuration
    PC1     ------> ADC1_IN7
    PC2     ------> ADC1_IN8
    PC3     ------> ADC1_IN9
    PA1     ------> ADC1_IN2
    PA7     ------> ADC1_IN15
    PB0     ------> ADC1_IN11
    PB1     ------> ADC1_IN12
    */
    GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* System interrupt init*/
    HAL_NVIC_SetPriority(ADC1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(ADC1_IRQn);
  /* USER CODE BEGIN ADC1_MspInit 1 */

  /* USER CODE END ADC1_MspInit 1 */
  }

}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef* hadc)
{

  if(hadc->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspDeInit 0 */

  /* USER CODE END ADC1_MspDeInit 0 */
    /* Peripheral clock disable */
    __ADC1_CLK_DISABLE();

    /**ADC1 GPIO Configuration
    PC1     ------> ADC1_IN7
    PC2     ------> ADC1_IN8
    PC3     ------> ADC1_IN9
    PA1     ------> ADC1_IN2
    PA7     ------> ADC1_IN15
    PB0     ------> ADC1_IN11
    PB1     ------> ADC1_IN12
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);

    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_1|GPIO_PIN_7);

    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_0|GPIO_PIN_1);

    /* Peripheral interrupt DeInit*/
    HAL_NVIC_DisableIRQ(ADC1_IRQn);

  /* USER CODE BEGIN ADC1_MspDeInit 1 */

  /* USER CODE END ADC1_MspDeInit 1 */
  }

}

void HAL_DAC_MspInit(DAC_HandleTypeDef* hdac)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(hdac->Instance==DAC)
  {
  /* USER CODE BEGIN DAC_MspInit 0 */

  /* USER CODE END DAC_MspInit 0 */
    /* Peripheral clock enable */
    __DAC1_CLK_ENABLE();

    /**DAC GPIO Configuration
    PA4     ------> DAC_OUT1
    */
    GPIO_InitStruct.Pin = GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* System interrupt init*/
    HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
  /* USER CODE BEGIN DAC_MspInit 1 */

  /* USER CODE END DAC_MspInit 1 */
  }

}

void HAL_DAC_MspDeInit(DAC_HandleTypeDef* hdac)
{

  if(hdac->Instance==DAC)
  {
  /* USER CODE BEGIN DAC_MspDeInit 0 */

  /* USER CODE END DAC_MspDeInit 0 */
    /* Peripheral clock disable */
    __DAC1_CLK_DISABLE();

    /**DAC GPIO Configuration
    PA4     ------> DAC_OUT1
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_4);

    /* Peripheral interrupt DeInit*/
  /* USER CODE BEGIN DAC:TIM6_DAC_IRQn disable */
    /**
    * Uncomment the line below to disable the "TIM6_DAC_IRQn" interrupt
    * Be aware, disabling shared interrupt may affect other IPs
    */
    /* HAL_NVIC_DisableIRQ(TIM6_DAC_IRQn); */
  /* USER CODE END DAC:TIM6_DAC_IRQn disable */

  /* USER CODE BEGIN DAC_MspDeInit 1 */

  /* USER CODE END DAC_MspDeInit 1 */
  }

}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(htim_base->Instance==TIM1)
  {
  /* USER CODE BEGIN TIM1_MspInit 0 */

  /* USER CODE END TIM1_MspInit 0 */
    /* Peripheral clock enable */
    __TIM1_CLK_ENABLE();

    /**TIM1 GPIO Configuration
    PA6     ------> TIM1_BKIN
    PA8     ------> TIM1_CH1
    PA9     ------> TIM1_CH2
    PA10     ------> TIM1_CH3
    PA12     ------> TIM1_ETR
    */
    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF6_TIM1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF6_TIM1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF11_TIM1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* System interrupt init*/
    HAL_NVIC_SetPriority(TIM1_BRK_TIM15_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM1_BRK_TIM15_IRQn);
  /* USER CODE BEGIN TIM1_MspInit 1 */

  /* USER CODE END TIM1_MspInit 1 */
  }
  else if(htim_base->Instance==TIM2)
  {
  /* USER CODE BEGIN TIM2_MspInit 0 */

  /* USER CODE END TIM2_MspInit 0 */
    /* Peripheral clock enable */
    __TIM2_CLK_ENABLE();

    /**TIM2 GPIO Configuration
    PA5     ------> TIM2_CH1
    PB10     ------> TIM2_CH3
    PB3     ------> TIM2_CH2
    */
	  // Hall sensors
    //GPIO_InitStruct.Pin = GPIO_PIN_5;
    //GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    //GPIO_InitStruct.Pull = GPIO_NOPULL;
    //GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    //GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
    //HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
//
    //GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_3;
    //GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    //GPIO_InitStruct.Pull = GPIO_NOPULL;
    //GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    //GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
    //HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN TIM2_MspInit 1 */

  /* USER CODE END TIM2_MspInit 1 */
  }
  else if(htim_base->Instance==TIM6)
  {
  /* USER CODE BEGIN TIM6_MspInit 0 */

  /* USER CODE END TIM6_MspInit 0 */
    /* Peripheral clock enable */
    __TIM6_CLK_ENABLE();
  /* System interrupt init*/
    HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
  /* USER CODE BEGIN TIM6_MspInit 1 */

  /* USER CODE END TIM6_MspInit 1 */
  }
  else if(htim_base->Instance==TIM15)
  {
	/* USER CODE BEGIN TIM15_MspInit 0 */

	  /* USER CODE END TIM15_MspInit 0 */
	  /* Peripheral clock enable */
	  __TIM15_CLK_ENABLE();

	  HAL_NVIC_SetPriority(TIM15_IRQn, 1, 0);
	  HAL_NVIC_EnableIRQ(TIM15_IRQn);
	  /* USER CODE BEGIN TIM16_MspInit 1 */

	  /* USER CODE END TIM16_MspInit 1 */
  }
  else if(htim_base->Instance==TIM16)
  {
  /* USER CODE BEGIN TIM16_MspInit 0 */

  /* USER CODE END TIM16_MspInit 0 */
    /* Peripheral clock enable */
    __TIM16_CLK_ENABLE();

    /**TIM16 GPIO Configuration
    PB4     ------> TIM16_CH1
    */
    GPIO_InitStruct.Pin = GPIO_PIN_4;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;//GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    //GPIO_InitStruct.Alternate = GPIO_AF1_TIM16;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN TIM16_MspInit 1 */

  /* USER CODE END TIM16_MspInit 1 */
  }

}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* htim_base)
{

  if(htim_base->Instance==TIM1)
  {
  /* USER CODE BEGIN TIM1_MspDeInit 0 */

  /* USER CODE END TIM1_MspDeInit 0 */
    /* Peripheral clock disable */
    __TIM1_CLK_DISABLE();

    /**TIM1 GPIO Configuration
    PA6     ------> TIM1_BKIN
    PA8     ------> TIM1_CH1
    PA9     ------> TIM1_CH2
    PA10     ------> TIM1_CH3
    PA12     ------> TIM1_ETR
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_6|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10
                          |GPIO_PIN_12);

    /* Peripheral interrupt DeInit*/
    HAL_NVIC_DisableIRQ(TIM1_BRK_TIM15_IRQn);

  /* USER CODE BEGIN TIM1_MspDeInit 1 */

  /* USER CODE END TIM1_MspDeInit 1 */
  }
  else if(htim_base->Instance==TIM2)
  {
  /* USER CODE BEGIN TIM2_MspDeInit 0 */

  /* USER CODE END TIM2_MspDeInit 0 */
    /* Peripheral clock disable */
    __TIM2_CLK_DISABLE();

    /**TIM2 GPIO Configuration
    PA5     ------> TIM2_CH1
    PB10     ------> TIM2_CH3
    PB3     ------> TIM2_CH2
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_5);

    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_10|GPIO_PIN_3);

  /* USER CODE BEGIN TIM2_MspDeInit 1 */

  /* USER CODE END TIM2_MspDeInit 1 */
  }
  else if(htim_base->Instance==TIM6)
  {
  /* USER CODE BEGIN TIM6_MspDeInit 0 */

  /* USER CODE END TIM6_MspDeInit 0 */
    /* Peripheral clock disable */
    __TIM6_CLK_DISABLE();

    /* Peripheral interrupt DeInit*/
  /* USER CODE BEGIN TIM6:TIM6_DAC_IRQn disable */
    /**
    * Uncomment the line below to disable the "TIM6_DAC_IRQn" interrupt
    * Be aware, disabling shared interrupt may affect other IPs
    */
    /* HAL_NVIC_DisableIRQ(TIM6_DAC_IRQn); */
  /* USER CODE END TIM6:TIM6_DAC_IRQn disable */

  /* USER CODE BEGIN TIM6_MspDeInit 1 */

  /* USER CODE END TIM6_MspDeInit 1 */
  }
  else if(htim_base->Instance==TIM16)
  {
  /* USER CODE BEGIN TIM16_MspDeInit 0 */

  /* USER CODE END TIM16_MspDeInit 0 */
    /* Peripheral clock disable */
    __TIM16_CLK_DISABLE();

    /**TIM16 GPIO Configuration
    PB4     ------> TIM16_CH1
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_4);

  /* USER CODE BEGIN TIM16_MspDeInit 1 */

  /* USER CODE END TIM16_MspDeInit 1 */
  }

}

void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(huart->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspInit 0 */

  /* USER CODE END USART2_MspInit 0 */
    /* Peripheral clock enable */
    __USART2_CLK_ENABLE();

    /**USART2 GPIO Configuration
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* System interrupt init*/
    HAL_NVIC_SetPriority(USART2_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
  /* USER CODE BEGIN USART2_MspInit 1 */

  /* USER CODE END USART2_MspInit 1 */
  }

}

void HAL_UART_MspDeInit(UART_HandleTypeDef* huart)
{

  if(huart->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspDeInit 0 */

  /* USER CODE END USART2_MspDeInit 0 */
    /* Peripheral clock disable */
    __USART2_CLK_DISABLE();

    /**USART2 GPIO Configuration
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2|GPIO_PIN_3);

    /* Peripheral interrupt DeInit*/
    HAL_NVIC_DisableIRQ(USART2_IRQn);

  /* USER CODE BEGIN USART2_MspDeInit 1 */

  /* USER CODE END USART2_MspDeInit 1 */
  }

}

/* USER CODE BEGIN 1 */
void HAL_TIM_Encoder_MspInit(TIM_HandleTypeDef *htim)
{ 
	if (htim->Instance == TIM2)
	{ 
		__TIM2_CLK_ENABLE();
 
		__GPIOA_CLK_ENABLE();
		__GPIOB_CLK_ENABLE();
		
		GPIO_InitTypeDef GPIO_InitStructureA;
		GPIO_InitStructureA.Pin = GPIO_PIN_15;
		GPIO_InitStructureA.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStructureA.Pull = GPIO_NOPULL;	
		GPIO_InitStructureA.Speed = GPIO_SPEED_HIGH;
		GPIO_InitStructureA.Alternate = GPIO_AF1_TIM2;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStructureA);
	
		GPIO_InitTypeDef GPIO_InitStructureB;
		GPIO_InitStructureB.Pin = GPIO_PIN_3;
		GPIO_InitStructureB.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStructureB.Pull = GPIO_NOPULL;
		GPIO_InitStructureB.Speed = GPIO_SPEED_HIGH;
		GPIO_InitStructureB.Alternate = GPIO_AF1_TIM2;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStructureB);
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

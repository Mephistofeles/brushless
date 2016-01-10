/**
 ******************************************************************************
 * @file    main_F302.c
 * @author  System lab - Automation and Motion control team
 * @version V1.0.0
 * @date    06-July-2015
 * @brief   This file provides a set of functions needed to configure STM32 MCU.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
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

/* USER CODE BEGIN Includes */
#include "6Step_Lib.h"
#include <stdbool.h>
#include <arm_math.h>
#include "l6230.h"
#include "stm32f3xx_nucleo.h"
#include "encoder.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

DAC_HandleTypeDef hdac;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim15;
TIM_HandleTypeDef htim16;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
float degree = 0;
SPI_HandleTypeDef nucleo_Spi;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_DAC_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM15_Init(void);
static void MX_TIM16_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

void SVPWM_run(float a, float m)
{
	float Ualpha, Ubeta;
	float X, Y, Z;
	float PWMa, PWMb, PWMc;
	uint8_t sector;

	//Ualpha = m * cos(a);
	//Ubeta = m * sin(a);
	
	float angle = a;
	if (angle >= 180.0f) angle -= 360.0f;
		
	arm_sin_cos_f32(angle, &Ubeta, &Ualpha);

	X = Ubeta;
	Y = Ubeta * 0.5f + Ualpha * 0.8660254f; //(Ubeta + Ualpha*sqrt(3))/2
	Z = X - Y;				//(Ubeta - Ualpha*sqrt(3))/2

	if (a < 60) sector = 0;
	else if (a < 120) sector = 1;
	else if (a < 180) sector = 2;
	else if (a < 240) sector = 3;
	else if (a < 300) sector = 4;
	else sector = 5;

	//switch (sector) {
	//case 0:
	//case 3:
		//PWMa = Y;
		//PWMb = X + Z;
		//PWMc = -Y;
		//break;
	//case 1:
	//case 4:
		//PWMa = Y - Z;
		//PWMb = X;
		//PWMc = -X;
		//break;
	//case 2:
	//case 5:
		//PWMa = -Z;
		//PWMb = Z;
		//PWMc = -(Y + X);
		//break;
	//default:
		//break;
	//}
	
	uint8_t restricted_angle = (uint16_t)a % 60;
	
	float restricted_cos = arm_cos_f32((restricted_angle + 30) * 0.0174532925f);
	float restricted_sin = arm_sin_f32((restricted_angle) * 0.0174532925f);
	
	float t1 = 0.8660254f * m * restricted_cos;
	float t2 = 0.8660254f * m * restricted_sin;
	float t0 = 1 - t1 - t2;
	
	uint16_t phase_U_enable_duty_cycle = 0;
	uint16_t phase_V_enable_duty_cycle = 0;
	uint16_t phase_W_enable_duty_cycle = 0;
	
	
	
	switch (sector) {
	case 0:
		phase_U_enable_duty_cycle = (t1 + t2) * 719;
		phase_V_enable_duty_cycle = t2 * 719;
		phase_W_enable_duty_cycle = 0;
		break;
	case 1:
		phase_U_enable_duty_cycle = t1 * 719;
		phase_V_enable_duty_cycle = (t1 + t2) * 719;
		phase_W_enable_duty_cycle = 0;
		break;
	case 2:
		phase_U_enable_duty_cycle = 0;
		phase_V_enable_duty_cycle = (t1 + t2) * 719;
		phase_W_enable_duty_cycle = t2 * 719;
		break;
	case 3:
		phase_U_enable_duty_cycle = 0;
		phase_V_enable_duty_cycle = t1 * 719;
		phase_W_enable_duty_cycle = (t1 + t2) * 719;
		break;
	case 4:
		phase_U_enable_duty_cycle = t2 * 719;
		phase_V_enable_duty_cycle = 0;
		phase_W_enable_duty_cycle = (t1 + t2) * 719;
		break;
	case 5:
		phase_U_enable_duty_cycle = (t1 + t2) * 719;
		phase_V_enable_duty_cycle = 0;
		phase_W_enable_duty_cycle = t1 * 719;
		break;
	default:
		break;
	}
	
	//arm_inv_clarke_f32(Ualpha, Ubeta, &PWMa, &PWMb);
	//
	//PWMc =  -0.5 * Ualpha - (float32_t) 0.8660254039 * Ubeta;
	
	//uint16_t phase_U_enable_duty_cycle = 350 + PWMa * 300;
	//uint16_t phase_V_enable_duty_cycle = 350 + PWMb * 300;
	//uint16_t phase_W_enable_duty_cycle = 350 + PWMc * 300;
	
	uint8_t phase_U_direction = PWMa > 0;
	uint8_t phase_V_direction = PWMb > 0;
	uint8_t phase_W_direction = PWMc > 0;	
	
	L6230_HFTIM_DC_CH1(phase_U_enable_duty_cycle);
	L6230_HFTIM_DC_CH2(phase_V_enable_duty_cycle);
	L6230_HFTIM_DC_CH3(phase_W_enable_duty_cycle);
}

static void SPIx_MspInit(SPI_HandleTypeDef *hspi)
{
	GPIO_InitTypeDef  GPIO_InitStruct;

	  /*** Configure the GPIOs ***/
	  /* Enable GPIO clock */
	NUCLEO_SPIx_SCK_GPIO_CLK_ENABLE();
	NUCLEO_SPIx_MISO_MOSI_GPIO_CLK_ENABLE();

	  /* Configure SPI SCK */
	GPIO_InitStruct.Pin = NUCLEO_SPIx_SCK_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull  = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
	GPIO_InitStruct.Alternate = NUCLEO_SPIx_SCK_AF;
	HAL_GPIO_Init(NUCLEO_SPIx_SCK_GPIO_PORT, &GPIO_InitStruct);

	  /* Configure SPI MISO and MOSI */
	GPIO_InitStruct.Pin = NUCLEO_SPIx_MOSI_PIN;
	GPIO_InitStruct.Alternate = NUCLEO_SPIx_MISO_MOSI_AF;
	GPIO_InitStruct.Pull  = GPIO_PULLDOWN;
	HAL_GPIO_Init(NUCLEO_SPIx_MISO_MOSI_GPIO_PORT, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = NUCLEO_SPIx_MISO_PIN;
	HAL_GPIO_Init(NUCLEO_SPIx_MISO_MOSI_GPIO_PORT, &GPIO_InitStruct);

	  /*** Configure the SPI peripheral ***/
	  /* Enable SPI clock */
	NUCLEO_SPIx_CLK_ENABLE();
}

static void SPI_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStruct;

	// Chip Select
	GPIO_InitStruct.Pin = GPIO_PIN_8;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
	
	if (HAL_SPI_GetState(&nucleo_Spi) == HAL_SPI_STATE_RESET)
	{
	  /* SPI Config */
		nucleo_Spi.Instance = NUCLEO_SPIx;
		  /* SPI baudrate is set to 8 MHz maximum (PCLKx/SPI_BaudRatePrescaler = 32/4 = 8 MHz)
		   to verify these constraints:
			  - ST7735 LCD SPI interface max baudrate is 15MHz for write and 6.66MHz for read
				Since the provided driver doesn't use read capability from LCD, only constraint
				on write baudrate is considered.
			  - SD card SPI interface max baudrate is 25MHz for write/read
			  - PCLK1 max frequency is 32 MHz
			  - PCLK2 max frequency is 64 MHz
		   */
		nucleo_Spi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
		nucleo_Spi.Init.Direction = SPI_DIRECTION_2LINES;
		nucleo_Spi.Init.CLKPhase = SPI_PHASE_2EDGE;
		nucleo_Spi.Init.CLKPolarity = SPI_POLARITY_LOW;
		nucleo_Spi.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLED;
		nucleo_Spi.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
		nucleo_Spi.Init.CRCPolynomial = 7;
		nucleo_Spi.Init.DataSize = SPI_DATASIZE_16BIT;
		nucleo_Spi.Init.FirstBit = SPI_FIRSTBIT_MSB;
		nucleo_Spi.Init.NSS = SPI_NSS_SOFT;
		nucleo_Spi.Init.TIMode = SPI_TIMODE_DISABLED;
		nucleo_Spi.Init.NSSPMode = SPI_NSS_PULSE_DISABLED;
		nucleo_Spi.Init.Mode = SPI_MODE_MASTER;

		SPIx_MspInit(&nucleo_Spi);
		HAL_SPI_Init(&nucleo_Spi);
	}
}


uint8_t parity(uint16_t frame)
{
	uint8_t t = (uint8_t)frame ^ frame >> 8;
	t ^= t >> 4;
	t ^= t >> 2;
	t ^= t >> 1;
	return t & 1;
}

int main(void)
{
  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	  /* Configure the system clock */
	SystemClock_Config();

	  /* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_ADC1_Init();
	MX_DAC_Init();
	MX_TIM1_Init(); // 3-phase PWM
	MX_TIM2_Init(); // Encoder
	MX_TIM6_Init(); // Reference current PWM
	MX_TIM15_Init();
	MX_TIM16_Init(); // Low frequency task
	MX_USART2_UART_Init();

	  /* USER CODE BEGIN 2 */
	  /* ****************************************************************************
	  ==============================================================================
				###### This function initializes 6-Step lib ######
	  ==============================================================================
	  **************************************************************************** */
	MC_SixStep_INIT();
	/****************************************************************************/
	/* USER CODE END 2 */


	  /*! **************************************************************************
	  ==============================================================================
				###### How to use the 6Step FW Example project ######
	  ==============================================================================
	  This workspace contains the middleware layer with Motor Control library to drive
	  a motor connected on X-Nucleo board performing a 6-step control algorithm
	  allowing the motor speed regulation through a potentiometer. The 6-step algorithm
	  is based on 1shunt current sensing mode and sensorless algorithm for bEmf detection.
	  The workspace is provided for STM32Fxx-Nucleo in four different configurations,
	  normal, demo, comm mode, boot mode. The "normal" mode waits the blue button event
	  to start the motor, the "demo" mode starts and stop the motor automatically, the
	  "comm" mode enables the communication protocol with external PC terminal and the
	  "boot" mode enables the FW for external boot loader.

		 A list of APIs is provided to send command to 6Step lib, for instance:

			(#)  MC_StartMotor() -> Start the motor

				(#)  MC_StoptMotor() -> Stop the motor

					(#)  MC_Set_Speed(...) -> Set the new motor speed

					  The MC_SixStep_param.h contains the full list of MC parameters
					  ****************************************************************************/

						/* Infinite loop */
	
	
	
	L6230_Start_PWM_generation();
	//MC_SixStep_Start_PWM_driving();
	MC_SixStep_Current_Reference_Start();
	MC_SixStep_Current_Reference_Setvalue(2000);

	
	HAL_GPIO_WritePin(GPIO_PORT_PHASE_ENABLE, GPIO_CH1_PHASE_U_ENABLE, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIO_PORT_PHASE_ENABLE, GPIO_CH2_PHASE_V_ENABLE, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIO_PORT_PHASE_ENABLE, GPIO_CH3_PHASE_W_ENABLE, GPIO_PIN_SET);
	

	//arm_pid_instance_f32 pid;
	//pid.Kp = 0.005f;
	//pid.Ki = 0;
	//pid.Kd = 0;
	//arm_pid_init_f32(&pid, 1);
	
	float speed = 0.001f;
	HAL_Delay(300);
	while (1)
	{
		//float ref = arm_pid_f32(&pid, 30 - get_speed());
		//
		//if (ref > 0.0001f) ref = 0.0001f;
		//
		//
		if (degree >= 360)
		{
			degree -= 360;	
		}
		if (speed < 4.0f)
		{
			speed *= 1.000005f;
		}
		else
		{
			speed *= 1.000005f;        
		}
			
		if (speed > 20.0f) speed = 20.0f;
//speed += ref;
//if (get_speed() < 10 && speed > 0.2f) speed = 0.2f;
//if (get_speed() > 10 && speed > 0.4f) speed = 0.4f;
		degree += speed;

		SVPWM_run(degree, 1);
	}
	
	//SPI_Init();
	//while (1)
	//{		
		//HAL_StatusTypeDef status = HAL_OK;
		//
		//uint16_t address = 0x3FFE;
			//
		//unsigned short frame = address | (1 << 14);
//
		//frame |= (parity(frame) << 15);			
			//
		//uint16_t readvalue = 0;
//
		//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
		//status = HAL_SPI_TransmitReceive(&nucleo_Spi, (uint8_t*) &frame, (uint8_t*) &readvalue, 1, NUCLEO_SPIx_TIMEOUT_MAX);
		//
		//uint16_t angle = readvalue  & 0x3FFF;
		//
		//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
	//}
	
	while (1)
	{
	}
}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
	HAL_RCC_OscConfig(&RCC_OscInitStruct);

	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
	HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);

	RCC_PeriphCLKInitTypeDef PeriphClkInit;
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2 | RCC_PERIPHCLK_TIM1 | RCC_PERIPHCLK_TIM15
								| RCC_PERIPHCLK_TIM16 | RCC_PERIPHCLK_ADC1;
	PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
	PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
	PeriphClkInit.Tim15ClockSelection = RCC_TIM15CLK_HCLK;
	PeriphClkInit.Tim16ClockSelection = RCC_TIM16CLK_HCLK;
	PeriphClkInit.Adc1ClockSelection = RCC_ADC1PLLCLK_DIV1;
	HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

	__SYSCFG_CLK_ENABLE();
}

/* ADC1 init function */
void MX_ADC1_Init(void)
{
		/**Common config
		*/
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC;
	hadc1.Init.Resolution = ADC_RESOLUTION12b;
	hadc1.Init.ScanConvMode = DISABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
	hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_TRGO;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.EOCSelection = EOC_SINGLE_CONV;
	hadc1.Init.LowPowerAutoWait = DISABLE;
	hadc1.Init.Overrun = OVR_DATA_OVERWRITTEN;
	HAL_ADC_Init(&hadc1);

		/**Configure Regular Channel
		*/
	ADC_ChannelConfTypeDef sConfig;
	sConfig.Channel = ADC_CHANNEL_12;
	sConfig.Rank = 1;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.SamplingTime = ADC_SAMPLETIME_19CYCLES_5;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	HAL_ADC_ConfigChannel(&hadc1, &sConfig);

}

/* DAC init function */
void MX_DAC_Init(void)
{
		/**DAC Initialization
		*/
	hdac.Instance = DAC;
	HAL_DAC_Init(&hdac);

		/**DAC channel OUT1 config
		*/
	DAC_ChannelConfTypeDef sConfig;
	sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
	sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
	HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1);

}

/* TIM1 init function */
void MX_TIM1_Init(void)
{
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 0;
	htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
	htim1.Init.Period = 719;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	HAL_TIM_Base_Init(&htim1);

	TIM_ClockConfigTypeDef sClockSourceConfig;
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig);

	HAL_TIM_PWM_Init(&htim1);

	TIM_ClearInputConfigTypeDef sClearInputConfig;
	sClearInputConfig.ClearInputSource = TIM_CLEARINPUTSOURCE_ETR;
	sClearInputConfig.ClearInputPolarity = TIM_CLEARINPUTPOLARITY_NONINVERTED;
	sClearInputConfig.ClearInputPrescaler = TIM_CLEARINPUTPRESCALER_DIV1;
	sClearInputConfig.ClearInputFilter = 0;
	HAL_TIM_ConfigOCrefClear(&htim1, &sClearInputConfig, TIM_CHANNEL_1);

	HAL_TIM_ConfigOCrefClear(&htim1, &sClearInputConfig, TIM_CHANNEL_2);

	HAL_TIM_ConfigOCrefClear(&htim1, &sClearInputConfig, TIM_CHANNEL_3);

	TIM_MasterConfigTypeDef sMasterConfig;
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
	sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
	HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig);

	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig; // Disable PWM when DIAG/EN is pulled low by the H-bridge (overcurrent protection)
	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_ENABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_LOW;
	sBreakDeadTimeConfig.BreakFilter = 0;
	sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
	sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_LOW;
	sBreakDeadTimeConfig.Break2Filter = 0;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig);
	
	TIM_OC_InitTypeDef sConfigOC;
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 575;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1);

	HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2);

	HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3);

}

void MX_TIM2_Init()
{			
	htim2.Instance = TIM2;
	htim2.Init.Period = 4095;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Prescaler = 0;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
 
	TIM_Encoder_InitTypeDef encoder;
	encoder.EncoderMode = TIM_ENCODERMODE_TI12;
 
	encoder.IC1Filter = 0x0;
	encoder.IC1Polarity = TIM_INPUTCHANNELPOLARITY_RISING;
	encoder.IC1Prescaler = TIM_ICPSC_DIV1;
	encoder.IC1Selection = TIM_ICSELECTION_DIRECTTI;
 
	encoder.IC2Filter = 0x0;
	encoder.IC2Polarity = TIM_INPUTCHANNELPOLARITY_FALLING;
	encoder.IC2Prescaler = TIM_ICPSC_DIV1;
	encoder.IC2Selection = TIM_ICSELECTION_DIRECTTI;
	
	if (HAL_TIM_Encoder_Init(&htim2, &encoder) != HAL_OK)
	{
		//Error_Handler();
	}
	
	if (HAL_TIM_Encoder_Start_IT(&htim2, TIM_CHANNEL_1) != HAL_OK)
	{
		//Error_Handler();
	}
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.Pin = GPIO_PIN_10;
	GPIO_InitStructure.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStructure.Pull = GPIO_NOPULL;	
	GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0x0F, 0x00);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);	
}

/* TIM6 init function */
void MX_TIM6_Init(void)
{
	htim6.Instance = TIM6;
	htim6.Init.Prescaler = 0;
	htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim6.Init.Period = 65535;
	HAL_TIM_Base_Init(&htim6);

	TIM_MasterConfigTypeDef sMasterConfig;
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig);

}

void MX_TIM15_Init(void)
{
	htim15.Instance = TIM15;
	htim15.Init.Period = 20000;
	htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim15.Init.Prescaler = 1999;
	htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV4;
	HAL_TIM_Base_Init(&htim15);
	HAL_TIM_Base_Start(&htim15);
}

/* TIM16 init function */
void MX_TIM16_Init(void)
{
	htim16.Instance = TIM16;
	htim16.Init.Prescaler = 0;
	htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim16.Init.Period = 1439;
	htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim16.Init.RepetitionCounter = 0;
	HAL_TIM_Base_Init(&htim16);

	HAL_TIM_PWM_Init(&htim16);

	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;
	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.BreakFilter = 0;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	HAL_TIMEx_ConfigBreakDeadTime(&htim16, &sBreakDeadTimeConfig);

	TIM_OC_InitTypeDef sConfigOC;
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 720;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	HAL_TIM_PWM_ConfigChannel(&htim16, &sConfigOC, TIM_CHANNEL_1);

}

/* USART2 init function */
void MX_USART2_UART_Init(void)
{
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 19200;
	huart2.Init.WordLength = UART_WORDLENGTH_9B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_ODD;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	huart2.Init.OneBitSampling = UART_ONEBIT_SAMPLING_DISABLED;
	huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	HAL_UART_Init(&huart2);
}

/** Configure pins as
		* Analog
		* Input
		* Output
		* EVENT_OUT
		* EXTI
*/
void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;

	  /* GPIO Ports Clock Enable */
	__GPIOC_CLK_ENABLE();
	__GPIOF_CLK_ENABLE();
	__GPIOA_CLK_ENABLE();
	__GPIOB_CLK_ENABLE();

	  /*Configure GPIO pin : PC13 */
	//GPIO_InitStruct.Pin = GPIO_PIN_13;
	//GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	//GPIO_InitStruct.Pull = GPIO_PULLUP;
	//HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	  /*Configure GPIO pin : PC4 */
	GPIO_InitStruct.Pin = GPIO_PIN_4;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	  /*Configure GPIO pin : PB2 */
	GPIO_InitStruct.Pin = GPIO_PIN_2;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	  /*Configure GPIO pin : PC7 */
	GPIO_InitStruct.Pin = GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	  /*Configure GPIO pins : PC10 PC11 PC12 */
	GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	  /* EXTI interrupt init*/
	//HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
	//HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);


}

void EXTI0_1_IRQHandler(void)
{
	int d = 10;
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

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

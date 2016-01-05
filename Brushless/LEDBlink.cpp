#include <stm32f3xx_hal.h>
#include <complex>

#ifdef __cplusplus
extern "C"
#endif
void SysTick_Handler(void)
{
	HAL_IncTick();
	HAL_SYSTICK_IRQHandler();
}

float g_Arg, g_Sin, g_Cos;

int main(void)
{
	HAL_Init();

	__GPIOB_CLK_ENABLE();
	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_InitStructure.Pin = GPIO_PIN_2;

	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
	GPIO_InitStructure.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);

	for (g_Arg = 0; ; g_Arg += 0.01F)
	{
		g_Sin = sinf(g_Arg);
		g_Cos = cosf(g_Arg);

		int totalCycles = 5000;
		int onCycles = (int)(totalCycles * (g_Sin + 1)) / 2;
		int offCycles = totalCycles - onCycles;
		
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
		for (int i = 0; i < onCycles; i++)
			asm("nop");
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
		for (int i = 0; i < offCycles; i++)
			asm("nop");
	}
}

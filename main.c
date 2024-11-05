#include "main.h"
#include "keypad.h"
#include "dac.h"
#include "lookup.h"

typedef enum {
	SQUARE,
	SINE,
	RAMP,
	TRIANGLE,
} wavetype;


static uint8_t FREQCOUNT = 1;				// incremnter depending on frequency
static uint8_t DUTY = 50;					// Duty cycle percentage initial = 50
static wavetype STATE = SQUARE;				// current wave type, inital = square
static uint16_t COUNT = 0;					// count for postion in wave, inital = 0

static uint8_t STAR = 10;					// * key = 10 on keypad
static uint8_t ZERO = 11;					// 0 key = 11 on keypad
static uint8_t POUND = 12;					// # key = 12 on keypad

const uint16_t MAX3V = 3722;				// max dac value of 3.3v (4095), 3v = 3722

int main(void) {
	HAL_Init();
	SystemClock_Config();
	DAC_init();               	            // Initialize DAC with SPI
	keypad_init();             	            // Initialize Keypad Interface
	int16_t key_press = -1;					// default key press = -1



	RCC->APB1ENR1 |= (RCC_APB1ENR1_TIM2EN);	// Turn on timer TIM2 RCC
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;
	TIM2->DIER |= (TIM_DIER_UIE);	        // Update Interrupt Enable/C.C. Interrupt Enable
	NVIC->ISER[0] = (1 << TIM2_IRQn);       // Enable interrupts for TIM2 in NVIC
	TIM2->ARR = 332;						// ARR value calculated from clock and interrupt time
	TIM2->SR &= ~(TIM_SR_UIF);
	TIM2->CR1 |= TIM_CR1_CEN;              	// Start the timer / counter
	TIM2->CR1 &= ~(TIM_CR1_DIR);        	// Count-Up Mode for TIM2
	__enable_irq();       					// Enable interrupts globally

	while (1) {
		while (keypad_get_button() == -1);		// button press debounce
		key_press = keypad_get_button();		// get button press
		while (keypad_get_button() != -1);		// button release debounce
		switch (key_press) {
			case -1: break;	// button not pressed.

			case 1: 		// 1 button: 100Hz
				FREQCOUNT = 1;
				break;

			case 2: 		// 2 button: 200Hz
				FREQCOUNT = 2;
				break;

			case 3: 		// 3 button: 300Hz
				FREQCOUNT = 3;
				break;

			case 4: 		// 4 button: 400Hz
				FREQCOUNT = 4;
				break;

			case 5: 		// 5 button: 500Hz
				FREQCOUNT = 5;
				break;

			case 6: 		// 6 button: sine wave
				STATE = SINE;
				break;

			case 7: 		// 7 button: triangle wave
				STATE = TRIANGLE;
				break;

			case 8: 		// 8 button: ramp wave
				STATE = RAMP;
				break;

			case 9: 		// 9 button: square wave
				STATE = SQUARE;
				break;

			case STAR: 		// * button: duty cycle - 10%
				if (DUTY >= 20) {
					DUTY -= 10;
				}
				break;

			case ZERO: 		// 0 button: duty cycle = 50%
				DUTY = 50;
				break;

			case POUND: 	// # button: duty cycle + 10%
				if (DUTY <= 80) {
					DUTY += 10;
				}
				break;

			default: break;
		}
	}

}

void TIM2_IRQHandler(void) {

	if (TIM2->SR & TIM_SR_UIF) {                     // End of timer interrupt flag check

		switch (STATE) {

			case SQUARE:
				if (COUNT < ((2400 * DUTY) / 100)) {
					// if Square wave high, write 3v (3722)
					DAC_write(MAX3V);
				}
				else {
					// if square wave low, write 0v (0)
					DAC_write(0);
				}
				break;

			case SINE:
				// write to the DAC the value of sine wave at current count in the wave
				DAC_write(sinArr[COUNT]);
				break;

			case TRIANGLE:
				// write to the DAC the value of the triangle wave at current count in wave
				DAC_write(triangleArr[COUNT]);
				break;

			case RAMP:
				// write to the DAC the value of the ramp wave at current count in wave
				DAC_write(rampArr[COUNT]);
				break;
		}
		COUNT += FREQCOUNT;				// increment count by amont depending on frequency
		if (COUNT > 2400) {				// if count goes over max, set back at 0 to reset
			COUNT = 0;
		}
		TIM2->SR &= ~(TIM_SR_UIF);                    // Clear the update interrupt flag


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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

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

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
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
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

#define clk		10
#define stb		11
#define oe		12

#define CLK 		GPIO_PIN_10
#define STROBE 		GPIO_PIN_11
#define OutputEn	GPIO_PIN_12

#define R0 0
#define G0 1
#define B0 2
#define R1 3
#define G1 4
#define B1 5

#define ZERO	0x06999996
#define ONE		0x0E444464
#define TWO		0x0F124896
#define THREE	0x0698E896
#define FOUR	0x0444F555
#define FIVE	0x0698611F	//backwards, works better
#define SIX		0x06997196
#define SEVEN	0x0112488F
#define EIGHT	0x06996996
#define NINE	0x0698E996

UART_HandleTypeDef huart3;

char RX_array[10];
int char_pos[10];
int line[16];


/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);

void set_time(void);

int main(void)
{
	int x = 0, y = 0, z = 0;
	int RX_index = 0;
	int test = 0x55555555;
	char ABCD = 0;

	char temp_char = 'F';

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* Configure the system clock */
	SystemClock_Config();

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_USART3_UART_Init();

	USART3->CR1 |= (1<<2);		//set RE, enable USART3 RX

	char_pos[0] = ZERO;
	char_pos[1] = ONE;
	char_pos[2] = TWO;
	char_pos[3] = THREE;
	char_pos[4] = FOUR;
	char_pos[5] = FIVE;
	char_pos[6] = SIX;

	//clear registers
	//***********************************************************
	GPIOA->ODR = 0x00001000;
	for (x = 0; x < 64; x++) //clock 32 times to load 0 to all registers
	{
	  GPIOA->ODR ^= CLK;
	}

	GPIOA->ODR ^= STROBE;
	GPIOA->ODR ^= STROBE;
	//***********************************************************

  while (1)
  {
	  //16 lines
	  for (y = 0; y < 16; y++)
	  {
		  GPIOA->ODR = 0x00001000  | (y << 6);

		  for (x = 0; x < 32; x++) //clock 32 times to load all registers
			{
			  if (y <= 7)
			  {
				  if (line[y] & (1<<(x))) GPIOA->BSRR = (1<<R0) | (1<<B1);
				  else GPIOA->BRR = (1<<R0) | (1<<B1);
			  }

			  else
			  {
				  if (line[y] & (1<<(x))) GPIOA->BSRR = (1<<G0) | (1<<G1) | (1<<B1);
				  else GPIOA->BRR = (1<<G0) | (1<<G1) | (1<<B1);
			  }

			  GPIOA->ODR ^= CLK;
			  GPIOA->ODR ^= CLK;
			}

			//GPIOA->ODR ^= STROBE;
			//GPIOA->ODR ^= STROBE;

			GPIOA->BSRR = (1<<stb);
			for (z = 0; z < 1; z++); //apparently some of the logic has setup time
			GPIOA->BRR = (1<<stb);

			GPIOA->BRR = (1<<oe);		//output enable, Low active, clear bit 12
			//HAL_Delay(0);
			for (z = 0; z < 25; z++);
			GPIOA->BSRR = (1<<oe);	//set bit 12

			//Check USART3
			if (USART3->ISR & (1<<5))
			{
				temp_char = USART3->RDR;

				if ((temp_char == 'R') | (RX_index >= 1))
				{
					RX_array[RX_index] = temp_char;
					RX_index++;
					temp_char = 0;
					if (RX_index >= 10)
					{
						RX_index = 0;
						set_time();
					}

				}

				//TX data
				//USART3->CR1 |= (1<<3);	//set TE
				//USART3->TDR = temp_char;		//load TX register
				//HAL_Delay(500);
			}

	  }

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART3;
  PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3 
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7 
                          |GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11 
                          |GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 PA2 PA3 
                           PA4 PA5 PA6 PA7 
                           PA8 PA9 PA10 PA11 
                           PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3 
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7 
                          |GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11 
                          |GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void set_time(void)
{
	int a = 0;
	int b = 0;

	b = ((RX_array[4] - 48) * 10) + (RX_array[5] - 48);	//actual hours in military time
	b += 5; //add 5 instead of subtract 7

	if (b >= 25) b -= 24;
	if (b >= 13) b -= 12;	//correct for military time

	if (b >= 10)
		{
		b -= 10;
		RX_array[4] = '1';

		}
	else RX_array[4] = 0;

	RX_array[5] = b + 48;

	for (a = 4; a <= 9; a++)
	{
		switch (RX_array[a])
		{
		case '0':
			char_pos[a-4] = ZERO;
			break;
		case '1':
			char_pos[a-4] = ONE;
			break;
		case '2':
			char_pos[a-4] = TWO;
			break;
		case '3':
			char_pos[a-4] = THREE;
			break;
		case '4':
			char_pos[a-4] = FOUR;
			break;
		case '5':
			char_pos[a-4] = FIVE;
			break;
		case '6':
			char_pos[a-4] = SIX;
			break;
		case '7':
			char_pos[a-4] = SEVEN;
			break;
		case '8':
			char_pos[a-4] = EIGHT;
			break;
		case '9':
			char_pos[a-4] = NINE;
			break;
		default:
			char_pos[a-4] = 0;
			break;
		}

	}

	line[0] = (char_pos[0] & (0x0000000F))  | (char_pos[1] & (0x0000000F)) << 5 | (char_pos[2] & (0x0000000F)) << 12
			| (char_pos[3] & (0x0000000F)) << 17 | (char_pos[4] & (0x0000000F)) << 23 | (char_pos[5] & (0x0000000F)) << 28;

	line[1] = ((char_pos[0] & (0x000000F0)) >> 4) | (char_pos[1] & (0x000000F0)) << 1 | (char_pos[2] & (0x000000F0)) << 8
			| (char_pos[3] & (0x000000F0)) << 13 | (char_pos[4] & (0x000000F0)) << 19 | (char_pos[5] & (0x000000F0)) << 24;

	line[2] = ((char_pos[0] & (0x00000F00)) >> 8) | (char_pos[1] & (0x00000F00)) >> 3 | (char_pos[2] & (0x00000F00)) << 4
			| (char_pos[3] & (0x00000F00)) << 9 | (char_pos[4] & (0x00000F00)) << 15 | (char_pos[5] & (0x00000F00)) << 20;

	line[3] = ((char_pos[0] & (0x0000F000)) >> 12) | (char_pos[1] & (0x0000F000)) >> 7 | (char_pos[2] & (0x0000F000))
			| (char_pos[3] & (0x0000F000)) << 5 | (char_pos[4] & (0x0000F000)) << 11 | (char_pos[5] & (0x0000F000)) << 16;

	line[4] = ((char_pos[0] & (0x000F0000)) >> 16) | (char_pos[1] & (0x000F0000)) >> 11 | (char_pos[2] & (0x000F0000)) >> 4
			| (char_pos[3] & (0x000F0000)) << 1 | (char_pos[4] & (0x000F0000)) << 7  | (char_pos[5] & (0x000F0000)) << 12;

	line[5] = ((char_pos[0] & (0x00F00000)) >> 20) | (char_pos[1] & (0x00F00000)) >> 15 | (char_pos[2] & (0x00F00000)) >> 8
			| (char_pos[3] & (0x00F00000)) >> 3 | (char_pos[4] & (0x00F00000)) << 3| (char_pos[5] & (0x00F00000)) << 8;

	line[6] = ((char_pos[0] & (0x0F000000)) >> 24) | (char_pos[1] & (0x0F000000)) >> 19 | (char_pos[2] & (0x0F000000)) >> 12
			| (char_pos[3] & (0x0F000000)) >> 7 | (char_pos[4] & (0x0F000000)) >> 1 | (char_pos[5] & (0x0F000000)) << 4;

	line[7] = ((char_pos[0] & (0xF0000000)) >> 28) | (char_pos[1] & (0xF0000000)) >> 23 | (char_pos[2] & (0xF0000000)) >> 16
			| (char_pos[3] & (0xF0000000)) >> 11 | (char_pos[4] & (0xF0000000)) >> 5 | (char_pos[5] & (0xF0000000)) << 1;



	line[8] = (char_pos[0] & (0x0000000F))  | (char_pos[1] & (0x0000000F)) << 5 | (char_pos[2] & (0x0000000F)) << 12
			| (char_pos[3] & (0x0000000F)) << 17 | (char_pos[4] & (0x0000000F)) << 23 | (char_pos[5] & (0x0000000F)) << 28;

	line[9] = ((char_pos[0] & (0x000000F0)) >> 4) | (char_pos[1] & (0x000000F0)) << 1 | (char_pos[2] & (0x000000F0)) << 8
			| (char_pos[3] & (0x000000F0)) << 13 | (char_pos[4] & (0x000000F0)) << 19 | (char_pos[5] & (0x000000F0)) << 24;

	line[10] = ((char_pos[0] & (0x00000F00)) >> 8) | (char_pos[1] & (0x00000F00)) >> 3 | (char_pos[2] & (0x00000F00)) << 4
			| (char_pos[3] & (0x00000F00)) << 9 | (char_pos[4] & (0x00000F00)) << 15 | (char_pos[5] & (0x00000F00)) << 20;

	line[11] = ((char_pos[0] & (0x0000F000)) >> 12) | (char_pos[1] & (0x0000F000)) >> 7 | (char_pos[2] & (0x0000F000))
			| (char_pos[3] & (0x0000F000)) << 5 | (char_pos[4] & (0x0000F000)) << 11 | (char_pos[5] & (0x0000F000)) << 16;

	line[12] = ((char_pos[0] & (0x000F0000)) >> 16) | (char_pos[1] & (0x000F0000)) >> 11 | (char_pos[2] & (0x000F0000)) >> 4
			| (char_pos[3] & (0x000F0000)) << 1 | (char_pos[4] & (0x000F0000)) << 7  | (char_pos[5] & (0x000F0000)) << 12;

	line[13] = ((char_pos[0] & (0x00F00000)) >> 20) | (char_pos[1] & (0x00F00000)) >> 15 | (char_pos[2] & (0x00F00000)) >> 8
			| (char_pos[3] & (0x00F00000)) >> 3 | (char_pos[4] & (0x00F00000)) << 3| (char_pos[5] & (0x00F00000)) << 8;

	line[14] = ((char_pos[0] & (0x0F000000)) >> 24) | (char_pos[1] & (0x0F000000)) >> 19 | (char_pos[2] & (0x0F000000)) >> 12
			| (char_pos[3] & (0x0F000000)) >> 7 | (char_pos[4] & (0x0F000000)) >> 1 | (char_pos[5] & (0x0F000000)) << 4;

	line[15] = ((char_pos[0] & (0xF0000000)) >> 28) | (char_pos[1] & (0xF0000000)) >> 23 | (char_pos[2] & (0xF0000000)) >> 16
			| (char_pos[3] & (0xF0000000)) >> 11 | (char_pos[4] & (0xF0000000)) >> 5 | (char_pos[5] & (0xF0000000)) << 1;

}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

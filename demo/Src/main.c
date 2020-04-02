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
  * COPYRIGHT(c) 2020 STMicroelectronics
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
#include "stm32f0xx_hal.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim6;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM6_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void ResetPatte(GPIO_TypeDef *Port,unsigned int NumPatte);
void ResetPortA(GPIO_TypeDef *Port);
void ResetPortB(GPIO_TypeDef *Port);
void SetPatte(GPIO_TypeDef *Port,unsigned int NumPatte);
void SetPortA(GPIO_TypeDef *Port);
void SetPortB(GPIO_TypeDef *Port);
void ReadLetterPortA(GPIO_TypeDef *Port, unsigned char Lettre[]);
void ReadLetterPortB(GPIO_TypeDef *Port, unsigned char Lettre[]);
void fdelay(volatile unsigned int compteur);

volatile unsigned int flag = 0;
unsigned char lettre_A[6] = {0x80,0xF6,0xF6,0xF6,0x80,0xFF};
unsigned char lettre_B[6] = {0x80,0xB6,0xB6,0xB6,0xC1,0xFF};
unsigned char lettre_C[6] = {0xC1,0x41,0x41,0x41,0x41,0xFF};
unsigned char lettre_D[6] = {0x80,0x41,0x41,0x41,0x41,0xFF};
unsigned char lettre_E[6] = {0x80,0xB6,0xB6,0xB6,0xB7,0xFF};
unsigned char lettre_F[6] = {0x80,0xF6,0xF6,0xF6,0xFE,0xFF};
unsigned char lettre_G[6] = {0xC1,0xBE,0x41,0xB6,0xB6,0xFF};
unsigned char lettre_H[6] = {0x80,0xF7,0xF7,0xF7,0x80,0xFF};
unsigned char lettre_I[6] = {0xFF,0xBE,0x80,0xBE,0xFF,0xFF};
unsigned char lettre_J[6] = {0xDE,0xBE,0xBE,0xC0,0xFE,0xFF};
unsigned char lettre_K[6] = {0x80,0xF7,0xEB,0xDD,0x41,0xFF};
unsigned char lettre_L[6] = {0x80,0xBF,0xBF,0xBF,0xBF,0xFF};
unsigned char lettre_M[6] = {0x80,0xFD,0xF3,0xFD,0x80,0xFF};
unsigned char lettre_N[6] = {0x80,0xF9,0xF7,0xCF,0x80,0xFF};
unsigned char lettre_O[6] = {0xC1,0x41,0x41,0x41,0xC1,0xFF};
unsigned char lettre_P[6] = {0x80,0xF6,0xF6,0xF6,0xF9,0xFF};
unsigned char lettre_Q[6] = {0xC1,0x41,0xAE,0xC1,0xBF,0xFF};
unsigned char lettre_R[6] = {0x80,0xF6,0xE6,0xD6,0xB9,0xFF};
unsigned char lettre_S[6] = {0xB9,0xB6,0xB6,0xB6,0xCE,0xFF};
unsigned char lettre_T[6] = {0xFE,0xFE,0x80,0xFE,0xFE,0xFF};
unsigned char lettre_U[6] = {0xC0,0xBF,0xBF,0xBF,0xC0,0xFF};
unsigned char lettre_W[6] = {0xC0,0xBF,0xC7,0xBF,0xC0,0xFF};
unsigned char lettre_X[6] = {0x9C,0xEB,0xF7,0xEB,0xC9,0xFF};
unsigned char lettre_Y[6] = {0xF8,0xF7,0x8F,0xF7,0xF8,0xFF};
unsigned char lettre_Z[6] = {0x9E,0xAE,0xB6,0xBA,0xBC,0xFF};
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

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
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */


	  if (flag==1)
	  {
		  ReadLetterPortA(GPIOA, lettre_M);
		  ReadLetterPortA(GPIOA, lettre_A);
		  ReadLetterPortA(GPIOA, lettre_R);
		  flag=0;
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

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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

/* TIM6 init function */
static void MX_TIM6_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;

  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 47999;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 99;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3 
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC0 PC1 PC2 PC3 
                           PC4 PC5 PC6 PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3 
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

}

/* USER CODE BEGIN 4 */
void ResetPatte(GPIO_TypeDef *Port,unsigned int NumPatte){
	Port->ODR|=(1<<NumPatte);
}

void ResetPortA(GPIO_TypeDef *Port){
	Port->ODR |= (0x00FF);
}

void ResetPortB(GPIO_TypeDef *Port){
	Port->ODR |= (0xFF00);
}

void SetPatte(GPIO_TypeDef *Port,unsigned int NumPatte){
	Port->ODR&=~(1<<NumPatte);
}

void SetPortA(GPIO_TypeDef *Port){
Port->ODR &= (~(0x00FF));
}

void SetPortB(GPIO_TypeDef *Port){
Port->ODR &= (~(0xFF00));
}

void ReadLetterPortA(GPIO_TypeDef *Port, unsigned char Lettre[]){
	int i;
	for(i=0;i<6;i++){
		Port->ODR &= (~(Lettre[i]));
		fdelay(767);
		Port->ODR |= (0x00FF);
	}
}

void ReadLetterPortB(GPIO_TypeDef *Port, unsigned char Lettre[]){
	int i;
	for(i=0;i<6;i++){
		Port->ODR &= (~(Lettre[i]));
		fdelay(767);
		Port->ODR |= (0xFF00);
	}
}

void fdelay(volatile unsigned int compteur)
{
  unsigned int count = 0; //inialisation compteur
  for(count; count<compteur; count++)
  {
 //incrémentation
  }
  count=0; //remise à zéro

}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	 if(GPIO_Pin == GPIO_PIN_15)   // Si c'est la patte 15 (PA15) qui a demandé interruption
   {
    /* Code a executé */
    flag=1;
   }

}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

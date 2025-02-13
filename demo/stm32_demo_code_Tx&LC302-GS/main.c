
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
  * COPYRIGHT(c) 2018 STMicroelectronics
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
#include "stm32f4xx_hal.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "flow_decode.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

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
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  PROTOCOL protocol = UPIXELS; //需解析哪种协议，319和306GS支持MAVLINK_PX4、MAVLINK_APM、MSP和UPIXELS协议，302GS支持MAVLINK_PX4_NO_TOF、MSP_NO_TOF和UPIXELS_NO_TOF协议

  while (1)
  {
		static int ret;
		static unsigned char ch;
		//HAL_Delay(100);
		//printf("running...\n");
		HAL_UART_Receive(&huart3, &ch, 1, 25);

		switch (protocol) {
			case MAVLINK_PX4_NO_TOF: //302GS使用的MAVLINK_PX4纯光流协议
				ret = px4notof_parse_char(ch);
				if(!ret){
					static float integrated_x = 0;
					static float integrated_y = 0;
					static uint8_t quality = 0;
					integrated_x = px4_flow_data.integrated_x;
					integrated_y = px4_flow_data.integrated_y;
					quality = px4_flow_data.quality;
					printf("integrated_x=%f,integrated_y=%f,quality=%d\n",integrated_x,integrated_y,quality);
				}
				break;			
			case MAVLINK_PX4: //319和306GS使用的MAVLINK_PX4协议
				ret = px4_parse_char(ch);
				if(!ret){
					static float integrated_x = 0;
					static float integrated_y = 0;
					static float distance = 0;
					static uint8_t quality = 0;
					integrated_x = px4_flow_data.integrated_x;
					integrated_y = px4_flow_data.integrated_y;
					distance = px4_flow_data.distance;
					quality = px4_flow_data.quality;
					printf("integrated_x=%f,integrated_y=%f,distance=%f,quality=%d\n",integrated_x,integrated_y,distance,quality);
					
					static uint16_t current_distance = 0;
					current_distance = px4_dis_data.current_distance;
					printf("current_distance=%d\n",current_distance);
				}
				break;	
			case MAVLINK_APM: //319和306GS使用的MAVLINK_APM协议
				ret = apm_parse_char(ch);
				if(!ret){
					static float flow_comp_x = 0;
					static float flow_comp_y = 0;
					static float ground_distance = 0;
					static uint8_t quality = 0;
					flow_comp_x = apm_flow_data.flow_comp_x;
					flow_comp_y = apm_flow_data.flow_comp_y;
					ground_distance = apm_flow_data.ground_distance;
					quality = apm_flow_data.quality;
					printf("flow_comp_x=%f,flow_comp_y=%f,ground_distance=%f,quality=%d\n",flow_comp_x,flow_comp_y,ground_distance,quality);

					static uint16_t current_distance = 0;
					current_distance = apm_dis_data.current_distance;
					printf("current_distance=%d\n",current_distance);
				}
				break;			
			case MSP_NO_TOF: //302GS使用的MSP纯光流协议
				ret = mspnotof_parse_char(ch);
				if(!ret){
					static uint8_t flow_quality = 0;
					static int32_t motionX = 0;
					static int32_t motionY = 0;
					flow_quality = msp_flow_data.flow_quality;
					motionX = msp_flow_data.motionX;
					motionY = msp_flow_data.motionY;
					printf("flow_quality=%d,motionX=%d,motionY=%d\n",flow_quality,motionX,motionY);				
				}
				break;
			case MSP: //319和306GS使用的MSP协议
				ret = msp_parse_char(ch);
				if(!ret){
					static uint8_t flow_quality = 0;
					static int32_t motionX = 0;
					static int32_t motionY = 0;
					flow_quality = msp_flow_data.flow_quality;
					motionX = msp_flow_data.motionX;
					motionY = msp_flow_data.motionY;
					printf("flow_quality=%d,motionX=%d,motionY=%d\n",flow_quality,motionX,motionY);		
					
					static uint8_t quality = 0;
					static uint32_t distance = 0;
					quality = msp_dis_data.quality;
					distance = msp_dis_data.distance;
					printf("quality=%d,distance=%d\n",quality,distance);
				}
				break;
			case UPIXELS_NO_TOF: //302GS使用的UPIXELS纯光流协议
				ret = upnotof_parse_char(ch);
				if(!ret){
					static int16_t flow_x_integral = 0;
					static int16_t flow_y_integral = 0;
					static uint8_t valid = 0;
					flow_x_integral = up_flow_data.flow_x_integral;
					flow_y_integral = up_flow_data.flow_y_integral;
					valid = up_flow_data.valid;
					printf("flow_x_integral=%d,flow_y_integral=%d,valid=%d\n",flow_x_integral,flow_y_integral,valid);
				}
				break;
			case UPIXELS: //319和306GS使用的UPIXELS协议
				ret = up_parse_char(ch);
				if(!ret){
					static int16_t flow_x_integral = 0;
					static int16_t flow_y_integral = 0;
					static uint16_t ground_distance = 0;
					static uint8_t valid = 0;
					static uint8_t tof_confidence = 0;
					flow_x_integral = up_data.flow_x_integral;
					flow_y_integral = up_data.flow_y_integral;
					ground_distance = up_data.ground_distance;
					valid = up_data.valid;
					tof_confidence = up_data.tof_confidence;
					printf("flow_x_integral=%d,flow_y_integral=%d,ground_distance=%d,valid=%d,tof_confidence=%d\n",flow_x_integral,flow_y_integral,ground_distance,valid,tof_confidence);
				}
				break;
			break;
			default:
				printf("Wrong Protocol\n");
			break;
		}
		
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

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

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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

/* USER CODE BEGIN 4 */

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

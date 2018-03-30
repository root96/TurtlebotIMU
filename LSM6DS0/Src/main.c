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
#include "stm32l4xx_hal.h"

/* USER CODE BEGIN Includes */

#include <limits.h>
#include "MadgwickAHRS.h"
#include "lsm6ds0_msg.pb.h"
#include "pb_encode.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

////////////////////////////////////////////////
/////////// akcelerometr i żyroskop ////////////
////////////////////////////////////////////////

#define LSM6DS0_ADRESS (0x6B << 1) // 1101011x
#define WHO_AM_I 0x0F
#define OUT_X_L_G 0x18 // m�odszy bit dla osi x , dla �yroskopu
#define OUT_X_L_XL 0x28 // m�odszy bit dla osi x, dla akcelerometru
#define LSM6DS0_GYRO_MULTI_READ (OUT_X_L_G | 0x80) // uruchomienie autoinkremetacji po rejestrach
#define LSM6DS0_ACC_MULTI_READ (OUT_X_L_XL | 0x80) // uruchomienie autoinkremetacji po rejestrach

// Rejestry �yroskopu
#define CTRL_REG1_G 0x10 // ustawienie cz�stotliwo�ci �yroskopu i skali pomiaru
uint8_t LSM6DS0_GYRO_952HZ_500DPS = 0xCB; // 11001011

#define CTRL_REG4 0x1E // ustawienie pobierania danych z yaw(odchylenie), pitch(nachylenie), roll(skr�canie)
uint8_t LSM6DS0_GYRO_ALL_AXIS_ENABLE = 0x38; // 00111000

// Rejestry akcelerometru
#define CTRL_REG5_XL 0x1F // adres rejestru od uruchomienie osi x,y,z
uint8_t LSM6DS0_ACC_ALL_AXIS_ENABLE = 0x38; // 00111000

// sta� warto�ci
#define LSM6DS0_ACC_RESOLUTION 2.0 // g
#define LSM6DS0_GYRO_RESOLUTION 500 // dps
#define PI 3.14159265358979323846
#define G 9.80665

// zmienne globalne
uint8_t DataGyro[6];
uint8_t DataAcc[6];

// zmienne dla akcelerometru
int16_t Xacc_m = 0;
int16_t Yacc_m = 0;
int16_t Zacc_m = 0;

float Xacc = 0;
float Yacc = 0;
float Zacc = 0;

// zmienne dla �yroskopu
int16_t Xgyro_m = 0;
int16_t Ygyro_m = 0;
int16_t Zgyro_m = 0;

float Xgyro = 0;
float Ygyro = 0;
float Zgyro = 0;

uint8_t who_am_i_ag;

////////////////////////////////////////////////
///////////////// magnetometr //////////////////
////////////////////////////////////////////////

#define LIS3MDL_ADRESS (0x1E << 1) //  0011100b
uint8_t who_am_i_m;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void) {

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
	MX_DMA_Init();
	MX_I2C1_Init();
	MX_USART2_UART_Init();

	/* USER CODE BEGIN 2 */

	HAL_I2C_Mem_Read(&hi2c1, LIS3MDL_ADRESS, WHO_AM_I, 1, &who_am_i_m, 1, 100);
	HAL_I2C_Mem_Read(&hi2c1, LSM6DS0_ADRESS, WHO_AM_I, 1, &who_am_i_ag, 1, 100);


	HAL_I2C_Mem_Write(&hi2c1, LSM6DS0_ADRESS, CTRL_REG1_G, 1,
			&LSM6DS0_GYRO_952HZ_500DPS, 1, 100);
	HAL_I2C_Mem_Write(&hi2c1, LSM6DS0_ADRESS, CTRL_REG4, 1,
			&LSM6DS0_GYRO_ALL_AXIS_ENABLE, 1, 100);

	/* USER CODE END 2 */

	lsm6ds0_msg msg_encoded = lsm6ds0_msg_init_zero;
	uint8_t buffer[100];
	buffer[0] = 0xFF;
	size_t message_lenght;

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		HAL_I2C_Mem_Read(&hi2c1, LSM6DS0_ADRESS, LSM6DS0_GYRO_MULTI_READ, 1,
				DataGyro, 6, 100);
		HAL_I2C_Mem_Read(&hi2c1, LSM6DS0_ADRESS, LSM6DS0_ACC_MULTI_READ, 1,
				DataAcc, 6, 100);

		Xgyro_m = ((DataGyro[1] << 8) | DataGyro[0]);
		Xgyro = (((float) Xgyro_m * LSM6DS0_GYRO_RESOLUTION) / (float) INT16_MAX)
						* (PI / 180);

		Ygyro_m = ((DataGyro[3] << 8) | DataGyro[2]);
		Ygyro = (((float) Ygyro_m * LSM6DS0_GYRO_RESOLUTION) / (float) INT16_MAX)
						* (PI / 180);

		Zgyro_m = ((DataGyro[5] << 8) | DataGyro[4]);
		Zgyro = (((float) Zgyro_m * LSM6DS0_GYRO_RESOLUTION) / (float) INT16_MAX)
						* (PI / 180);

		Xacc_m = ((DataAcc[1] << 8) | DataAcc[0]);
		Xacc = (((float) Xacc_m * LSM6DS0_ACC_RESOLUTION) / (float) INT16_MAX)
				* (G);

		Yacc_m = ((DataAcc[3] << 8) | DataAcc[2]);
		Yacc = (((float) Yacc_m * LSM6DS0_ACC_RESOLUTION) / (float) INT16_MAX)
				* (G);

		Zacc_m = ((DataAcc[5] << 8) | DataAcc[4]);
		Zacc = (((float) Zacc_m * LSM6DS0_ACC_RESOLUTION) / (float) INT16_MAX)
				* (G);

		// Madgwick filter
		MadgwickAHRSupdateIMU(Xgyro, Ygyro, Zgyro, Xacc, Yacc, Zacc);

		msg_encoded.acc_x = Xacc; msg_encoded.acc_y = Yacc; msg_encoded.acc_z = Zacc;
		msg_encoded.gyro_x = Xgyro; msg_encoded.gyro_y = Ygyro; msg_encoded.gyro_z = Zgyro;
		msg_encoded.q0 = q0; msg_encoded.q1 = q1; msg_encoded.q2 = q2; msg_encoded.q3 = q3;

		pb_ostream_t stream = pb_ostream_from_buffer(&buffer[2], sizeof(buffer)-2);
		pb_encode(&stream, lsm6ds0_msg_fields, &msg_encoded);
		message_lenght = stream.bytes_written;
		buffer[1] = (uint8_t)message_lenght;

		HAL_GPIO_WritePin(Green_LED_GPIO_Port, Green_LED_Pin, 1);
		HAL_UART_Transmit_DMA(&huart2, buffer, message_lenght+2);

		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

	}
	/* USER CODE END 3 */

}

/** System Clock Configuration
 */
void SystemClock_Config(void) {

	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_PeriphCLKInitTypeDef PeriphClkInit;

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = 16;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 1;
	RCC_OscInitStruct.PLL.PLLN = 10;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2
			| RCC_PERIPHCLK_I2C1;
	PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
	PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Configure the main internal regulator output voltage
	 */
	if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1)
			!= HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Configure the Systick interrupt time
	 */
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

	/**Configure the Systick
	 */
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	/* SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* I2C1 init function */
static void MX_I2C1_Init(void) {

	hi2c1.Instance = I2C1;
	hi2c1.Init.Timing = 0x00702991;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Configure Analogue filter
	 */
	if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE)
			!= HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Configure Digital filter
	 */
	if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

}

/* USART2 init function */
static void MX_USART2_UART_Init(void) {

	huart2.Instance = USART2;
	huart2.Init.BaudRate = 19200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

}

/** 
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {
	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE()
	;

	/* DMA interrupt init */
	/* DMA1_Channel6_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
	/* DMA1_Channel7_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);

}

/** Configure pins as 
 * Analog
 * Input
 * Output
 * EVENT_OUT
 * EXTI
 */
static void MX_GPIO_Init(void) {

	GPIO_InitTypeDef GPIO_InitStruct;

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOH_CLK_ENABLE()
	;
	__HAL_RCC_GPIOA_CLK_ENABLE()
	;
	__HAL_RCC_GPIOB_CLK_ENABLE()
	;

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(Green_LED_GPIO_Port, Green_LED_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : Green_LED_Pin */
	GPIO_InitStruct.Pin = Green_LED_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(Green_LED_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  None
 * @retval None
 */
void _Error_Handler(char * file, int line) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	while (1) {
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

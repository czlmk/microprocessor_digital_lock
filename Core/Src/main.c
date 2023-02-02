/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
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
#include "stm32l4s5i_iot01_hsensor.h"
#include <stdio.h>
#include <string.h>
#include "arm_math.h"
#include "stm32l4s5i_iot01_accelero.h"

#define ITM_Port32(n) (*((volatile unsigned long *) (0xE0000000+4*n)))
#define ARM_MATH_CM4
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
DAC_HandleTypeDef hdac1;

I2C_HandleTypeDef hi2c2;

UART_HandleTypeDef huart1;

uint8_t Rx_data;

float hsensor;
char strSensor[50];
char str2[20];
char *str = { 0 };
int open = 0;
int counter = 0;
int16_t acceleration_XYZ[3];
int16_t prev_acceleration_XYZ[3];
int iter = 0;
float is_accelerating = 0;
// 25x gravity on earth
float acc_thresh = 50 * 9.81;

int shaken = 0;
uint32_t stmsg = 1; // varible to see if printing starting msg is needed

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_DAC1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {

	if (open) {
		counter += 1;
		if (counter > 4) {
			counter = 0;
		}
	} else {
		// notify user through UART display that the board is not sleeping anymore
		str = "WakeUP from SLEEP\r\n";
		HAL_UART_Transmit(&huart1, (uint8_t*) str, strlen(str), HAL_MAX_DELAY);
		HAL_PWR_DisableSleepOnExit();
	}

}

/**
 * Prints the humidity sensor reading to the UART display.
 */
void StartTransmitHumidity(void) {
	/* USER CODE BEGIN StartTransmitTask */

	// UART status
	HAL_UART_Init(&huart1);
	/* Infinite loop */
	ITM_Port32(31) = 2;
	sprintf(strSensor, "Humidity: %.2d\r\n", (int) hsensor);
	HAL_UART_Transmit(&huart1, (uint8_t*) strSensor,
			(uint16_t) strlen(strSensor), 10000);

}

/**
 * Print acceleration to UART display.
 */
void StartTransmitAcceleration(void) {
	sprintf(strSensor, "Previous x: %.2d y: %.2d z: %.2d\r\n",
			(int) prev_acceleration_XYZ[0], (int) prev_acceleration_XYZ[1],
			(int) prev_acceleration_XYZ[2]);
	HAL_UART_Transmit(&huart1, (uint8_t*) strSensor,
			(uint16_t) strlen(strSensor), 10000);

	sprintf(strSensor, "Current x: %.2d y: %.2d z: %.2d\r\n",
			(int) acceleration_XYZ[0], (int) acceleration_XYZ[1],
			(int) acceleration_XYZ[2]);
	HAL_UART_Transmit(&huart1, (uint8_t*) strSensor,
			(uint16_t) strlen(strSensor), 10000);

}

/**
 * This method reads the humidity value from the board.
 */
void StartReadHumidity(void) {
	BSP_HSENSOR_Init();
	ITM_Port32(31) = 3;

	hsensor = BSP_HSENSOR_ReadHumidity();
}

/**
 * Handles the reading of the humidity sensor and optionally wakes up the board if the
 * humidity value is above a given threshold (37). Note that this function also drives
 * a count-up timer which above a threshold triggers a switch into low power mode.
 * If the humidity threshold is passed, the board is set as open and a noise is emitted.
 */
void readHumidity(void) {
	StartReadHumidity();
	int timer = 0;
	// The user has 30 sec to find how to open box
	while (hsensor < 37 && timer < 100) {
		StartReadHumidity();
		//StartTransmitHumidity();
		HAL_Delay((uint32_t) 200);
		timer = timer + 1;
		//sprintf(str2, "here: %.2d\r\n", (int) timer);
		//HAL_UART_Transmit(&huart1, (uint8_t*) str2, strlen(str2),
		//		HAL_MAX_DELAY);
	}

	// if the timer run out the box is closed
	if (timer >= 100) {
		open = 0;
		str = "SLEEP MODE\r\n";
		HAL_UART_Transmit(&huart1, (uint8_t*) str, strlen(str), HAL_MAX_DELAY);

		// switch to low power mode
		HAL_SuspendTick();

		HAL_PWR_EnableSleepOnExit();

		HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
	} else {
		open = 1;
	}

	// if the box is open a noise is emited
	if (open) {
		int timer2 = 0;
		while (timer2 < 100) { //timer2 < 1000
			for (uint32_t i = 0; i < 65; i++) {
				float32_t radians = 3.14 * i / 32.5;
				float32_t sin = arm_sin_f32(radians);
				uint32_t z = (uint32_t) (*(uint32_t*) &sin);
				HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, z);
				HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
			}

			timer2 = timer2 + 1;
		}

	}
}

/**
 * Threshold evaluator functions which determines whether the board has been shaken.
 */
void shake(void) {
	if ((acceleration_XYZ[0] != prev_acceleration_XYZ[0])
			&& (abs(acceleration_XYZ[0]) >= acc_thresh)
			&& (acceleration_XYZ[1] != prev_acceleration_XYZ[1])
			&& (abs(acceleration_XYZ[1]) >= acc_thresh)
			&& (acceleration_XYZ[2] != prev_acceleration_XYZ[2])
			&& (abs(acceleration_XYZ[2]) >= acc_thresh)) {
		shaken = 1;
	}
}

/**
 * Checks whether the board has been shaken, and handles the accelerometer readings.
 * This function handles the switch to low power mode in case of a detected movement of the board.
 */
void checkShaken(void) {
	BSP_ACCELERO_AccGetXYZ(acceleration_XYZ);
	if (iter > 0) {
		shake();
		// optionally print the read values
		//StartTransmitAcceleration();
	}
	iter++;

	// copy all values of current acceleration to other
	for (int i = 0; i < 3; i++) {
		prev_acceleration_XYZ[i] = acceleration_XYZ[i];
	}

	if (shaken == 1) {
		open = 0;
		int timer2 = 0;
		while (timer2 < 50) {
			// make a noise on exit
			for (uint32_t i = 0; i < 65; i++) {
				float32_t radians = 3.14 * i / 32.5;
				float32_t sin = arm_sin_f32(radians);
				uint32_t z = (uint32_t) (*(uint32_t*) &sin);
				HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, z);
				HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
			}

			timer2 = timer2 + 1;
		}
		HAL_Delay(500);
		timer2 = 0;
		while (timer2 < 50) {
			for (uint32_t i = 0; i < 65; i++) {
				float32_t radians = 3.14 * i / 32.5;
				float32_t sin = arm_sin_f32(radians);
				uint32_t z = (uint32_t) (*(uint32_t*) &sin);
				HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, z);
				HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
			}

			timer2 = timer2 + 1;
		}

		shaken = 0;

		// notify user that the board is entering a low power mode
		str = "SLEEP MODE \r\n";
		HAL_UART_Transmit(&huart1, (uint8_t*) str, strlen(str), HAL_MAX_DELAY);
		// enter the low power mode
		HAL_SuspendTick();

		HAL_PWR_EnableSleepOnExit();

		HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);

	}

}

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

	HAL_Init();

	MX_GPIO_Init();
	MX_I2C2_Init();
	MX_USART1_UART_Init();
	MX_DAC1_Init();
	BSP_ACCELERO_Init();

	uint32_t timestart = 0;	//for recording start time of record mode
	uint32_t fivesecstart = 0; // for recording starting time of single character
	uint32_t maintimer = 0; //record start time of deciding listen mode or record mode
	char morse[50];	//for strings to be printed

	char msg1[120];
	sprintf(msg1,
			"Record mode, 30 second start (once for ., twice for _, three times for pause, four times to exit):\r\n");
	char msg2[100];
	sprintf(msg2, " message end\r\n");
	char recorded[100]; //array to save recorded message
	sprintf(recorded, "Recorded message: \r\n");
	char startingmsg[100];
	sprintf(startingmsg,
			"Press once to record, press twice to see recorded message:\r\n");
	//breakout variable
	uint16_t breakout = 0;
	int checkCounter = 0;

	HAL_UART_Receive_IT(&huart1, &Rx_data, 1);

	// start in sleep mode
	str = "SLEEP MODE \r\n";
	HAL_UART_Transmit(&huart1, (uint8_t*) str, strlen(str), HAL_MAX_DELAY);

	HAL_SuspendTick();

	HAL_PWR_EnableSleepOnExit();

	HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);

	while (1) {
		//When wake up

		HAL_ResumeTick();

		str = "WakeUP from SLEEP\r\n";
		stmsg = 1;
		HAL_UART_Transmit(&huart1, (uint8_t*) str, strlen(str), HAL_MAX_DELAY);

		//Try to unlock
		readHumidity();

		if (!open) {
			continue;
		} else {

			// Box unlocked write now
			while (open) {
				if (checkCounter > 300) {
					checkShaken();
					checkCounter = 0;
				}
				checkCounter++;
				//print starting message
				if (stmsg == 1) {
					stmsg = 0;
					HAL_UART_Transmit(&huart1, (uint8_t*) startingmsg,
							(uint16_t) strlen(startingmsg), 100000);
				}
				//if pressed, if no more presses after 1 second go to record mode
				//if pressed one more time after 1 second go to listen mode(print msg)

				if (counter == 1) {
					maintimer = HAL_GetTick();

					while (1) {
//						  if (checkCounter > 300){
//							  checkShaken();
//							  checkCounter =0;
//						  }
						checkCounter++;

						if ((HAL_GetTick() - maintimer) > 1000) {
							//if counter == 1 record
							/////////////////////////////
							//when button is pressed once go into record mode, terminates after 30seconds
							//if button pressed twice go into listen mode to print message
							//within record mode, press once to start recording if no more presses after 5 seconds
							//record a dot, if pressed one more time record dash, if
							if (counter == 1) {
								//enter while loop that terminates after 30seconds
								HAL_UART_Transmit(&huart1, (uint8_t*) msg1,
										(uint16_t) strlen(msg1), 100000);
								timestart = HAL_GetTick();
								//reset counter
								counter = 0;

								//
								while (1) {

									//if button pressed one more time, within 1 seconds if no more presses add dot to data
									//if pressed one more time, add dash to data, if pressed one more time, add pause
									if (counter == 1) {
										fivesecstart = HAL_GetTick();
										// recording
										while (1) {
											checkShaken();

											//if 1 seconds pass reset counter and transmit and break
											if ((HAL_GetTick() - fivesecstart)
													> 1000) {
												// if counter = 1 add dot to data
												if (counter == 1) {
													strcpy(morse, ".");
													strcat(recorded, ".");

												}
												//if counter = 2 add dash to data
												else if (counter == 2) {
													strcpy(morse, "_");
													strcat(recorded, "_");

												}
												//if counter = 3 add pause as p to data
												else if (counter == 3) {
													strcpy(morse, " ");
													strcat(recorded, " ");

												} else if (counter == 4) {
													breakout = 1;
													counter = 0;
													break;
												}
												//transmit
												HAL_UART_Transmit(&huart1,
														(uint8_t*) morse,
														(uint16_t) strlen(
																morse), 100000);
												//reset counter
												counter = 0;
												break;
											}
										}
									} else if (breakout == 1) {

										break;
									}

									//terminate after 30seconds, reset counter
									else if ((HAL_GetTick() - timestart)
											> 30000) {
										//reset counter
										counter = 0;
										breakout = 1;
										break;
									}
								}
							}

							//setup breakout condition check
							else if (breakout == 1) {
								//reset breakout
								breakout = 0;
								counter = 0;
								strcat(recorded, "\r\n");
								HAL_UART_Transmit(&huart1, (uint8_t*) msg2,
										(uint16_t) strlen(msg2), 100000);
								stmsg = 1;
								break;
							}
							//////////////////////////////

							//if counter =2  listen
							else if (counter == 2) {
								HAL_UART_Transmit(&huart1, (uint8_t*) recorded,
										(uint16_t) strlen(recorded), 100000);
								counter = 0;
								stmsg = 1;
								break;
							}
						}
					}
				}
			}
		}
	}
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1)
			!= HAL_OK) {
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
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief DAC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_DAC1_Init(void) {

	/* USER CODE BEGIN DAC1_Init 0 */

	/* USER CODE END DAC1_Init 0 */

	DAC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN DAC1_Init 1 */

	/* USER CODE END DAC1_Init 1 */
	/** DAC Initialization
	 */
	hdac1.Instance = DAC1;
	if (HAL_DAC_Init(&hdac1) != HAL_OK) {
		Error_Handler();
	}
	/** DAC channel OUT1 config
	 */
	sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
	sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
	sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
	sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
	sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
	if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN DAC1_Init 2 */

	/* USER CODE END DAC1_Init 2 */

}

/**
 * @brief I2C2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C2_Init(void) {

	/* USER CODE BEGIN I2C2_Init 0 */

	/* USER CODE END I2C2_Init 0 */

	/* USER CODE BEGIN I2C2_Init 1 */

	/* USER CODE END I2C2_Init 1 */
	hi2c2.Instance = I2C2;
	hi2c2.Init.Timing = 0x10909CEC;
	hi2c2.Init.OwnAddress1 = 0;
	hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c2.Init.OwnAddress2 = 0;
	hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c2) != HAL_OK) {
		Error_Handler();
	}
	/** Configure Analogue filter
	 */
	if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE)
			!= HAL_OK) {
		Error_Handler();
	}
	/** Configure Digital filter
	 */
	if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C2_Init 2 */

	/* USER CODE END I2C2_Init 2 */

}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void) {

	/* USER CODE BEGIN USART1_Init 0 */

	/* USER CODE END USART1_Init 0 */

	/* USER CODE BEGIN USART1_Init 1 */

	/* USER CODE END USART1_Init 1 */
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 115200;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin : PC13 */
	GPIO_InitStruct.Pin = GPIO_PIN_13;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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


/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dac.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

// --> include all necessary headers for
// printf() redirection
// FreeRTOS related headers

#include "pid.h"
#include <stdio.h>
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <queue.h>

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

/* USER CODE BEGIN PV */
enum QueueStatus {
	QueueOK, QueueWriteProblem, QueueEmpty, QueueCantRead
};

uint8_t received_uart;
uint16_t measurement;
uint32_t desired_value = 0;
uint16_t control_value;

SemaphoreHandle_t mutex;
uint8_t queueError = QueueOK;
QueueHandle_t measured_queue;  // ADC values
QueueHandle_t desired_queue;   // desired values
QueueHandle_t control_queue;  // control signals

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int file, char *ptr, int len) {
	HAL_UART_Transmit(&huart2, (uint8_t *) ptr, len, 50);
	return len;
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if(huart->Instance == USART2) {
		HAL_UART_Receive_IT(&huart2, &received_uart, 1);
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
}

void measureTask(void *args) {
	TickType_t xLastWakeTime = xTaskGetTickCount();

	for (;;) {
		if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK){
				measurement = HAL_ADC_GetValue(&hadc1);
				HAL_ADC_Start(&hadc1);
			}

			BaseType_t xStatus = xQueueOverwrite(measured_queue, &measurement);

			if (xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE) {
				if (xStatus != pdPASS) {	// jesli sie nie udalo
					queueError = QueueWriteProblem;
				} else {					// jesli sie udalo
					queueError = QueueOK;
				}
				xSemaphoreGive(mutex);
			}
		vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10));		// 10 ms = 100 Hz
	}
}

void controlTask(void *args) {
    TickType_t xLastWakeTime = xTaskGetTickCount();

    pid_controller_t pid;

    pid.kp = 4.0f;
    pid.ki = 0.3f;
    pid.kd = 0.1f;

    pid.min = 0;
    pid.max = 4095;

    pid.e = 0;
    pid.e_prev = 0;
    pid.e_sum = 0;

    uint32_t _measured = 0;
    uint32_t _desired = 0;
    uint32_t _control = 0;

    for (;;) {
        if (xQueuePeek(measured_queue, &_measured, 100) == pdPASS) {}

        if (xQueuePeek(desired_queue, &_desired, 100) != pdPASS) {
            _desired = 0;
        }

        _control = pid_calc(&pid, _desired, _measured);

        HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
        HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, _control);

        if (xQueueOverwrite(control_queue, (void*)&_control) != pdPASS) {
            if (xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE) {
                queueError = QueueWriteProblem;
                xSemaphoreGive(mutex);
            }
        } else {
            if (xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE) {
                queueError = QueueOK;
                xSemaphoreGive(mutex);
            }
        }

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10));	// 10 ms = 100 Hz
    }
}


void commTask(void *args) {
	TickType_t xLastWakeTime = xTaskGetTickCount();
	uint32_t _measured = 0;
	uint32_t _desired = 0;
	uint32_t _control = 0;
	uint8_t _queueError = 0;

	for (;;) {
		if (xQueuePeek(measured_queue, &_measured, 100) == pdPASS) {}
		if (xQueuePeek(desired_queue, &_desired, 100) == pdPASS) {}
		if (xQueuePeek(control_queue, &_control, 100) == pdPASS) {}

		if (xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE) {
		    _queueError = queueError;
		    xSemaphoreGive(mutex);
		}

		printf("mv: %lu, dv: %lu, cs: %lu, queueError: %u\r\n", _measured, _desired, _control, _queueError);
		vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(500));  // 2 Hz = 500 ms
	}
}

void userTask(void *args) {
	TickType_t xLastWakeTime = xTaskGetTickCount();

	for (;;) {
		if (received_uart >= '0' && received_uart <= '8') {
			desired_value = (received_uart - '0') * 500;	// ascii to integer

			BaseType_t xStatus = xQueueOverwrite(desired_queue, &desired_value);

			if (xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE) {
				if (xStatus == pdPASS) {
					queueError = QueueOK;
				} else {
					queueError = QueueWriteProblem;
				}
				xSemaphoreGive(mutex);
			}
		}
		vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(100));  // 100 ms = 10 Hz
	}
}


/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	/* USER CODE BEGIN 1 */
	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

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
	MX_USART2_UART_Init();
	MX_ADC1_Init();
	MX_TIM6_Init();
	MX_DAC1_Init();
	/* USER CODE BEGIN 2 */

	HAL_ADC_Start(&hadc1);
	// enable UART receive in interrupt mode
	HAL_UART_Receive_IT(&huart2, &received_uart, 1);

	// --> create all necessary synchronization mechanisms
	mutex = xSemaphoreCreateMutex();
	measured_queue = xQueueCreate(1, sizeof(uint16_t));
	desired_queue = xQueueCreate(1, sizeof(uint16_t));
	control_queue = xQueueCreate(1, sizeof(uint16_t));
	// --> create all necessary tasks
	xTaskCreate(measureTask, "measureTask", configMINIMAL_STACK_SIZE * 4, NULL, 3, NULL);
	xTaskCreate(commTask, "commTask", configMINIMAL_STACK_SIZE * 4, NULL, 3, NULL);
	xTaskCreate(controlTask, "controlTask", configMINIMAL_STACK_SIZE * 4, NULL, 3, NULL);
	xTaskCreate(userTask, "userTask", configMINIMAL_STACK_SIZE * 4, NULL, 3, NULL);

	printf("Starting!\r\n");

	// --> start FreeRTOS scheduler
	vTaskStartScheduler();

	// --> start FreeRTOS scheduler

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
	RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 1;
	RCC_OscInitStruct.PLL.PLLN = 10;
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
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2
			| RCC_PERIPHCLK_ADC;
	PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
	PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
	PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_HSI;
	PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
	PeriphClkInit.PLLSAI1.PLLSAI1N = 8;
	PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
	PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
	PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
	PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_ADC1CLK;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		Error_Handler();
	}
	/** Configure the main internal regulator output voltage
	 */
	if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1)
			!= HAL_OK) {
		Error_Handler();
	}
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
	 tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

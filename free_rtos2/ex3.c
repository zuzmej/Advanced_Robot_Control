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
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

// --> include all necessary headers for
// printf() redirection
#include "stdio.h"
// FreeRTOS related headers
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
int _write(int file, char *ptr, int len) {
	HAL_UART_Transmit(&huart2, (uint8_t *) ptr, len, 50);
	return len;
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


// subtask 3:
typedef struct {
	uint16_t measurement;
	uint32_t counter;
} queue_data_t;

enum QueueStatus {
	QueueOK, QueueWriteProblem, QueueEmpty, QueueCantRead
};

enum QueueMessages {
	QueueMsgNoData, QueueMsgNewData, QueueMsgNewDataChange,
};

uint32_t counter = 0;
uint16_t measurement = 0;
uint8_t queueError = QueueOK;
SemaphoreHandle_t mutex;
QueueHandle_t queue;

// subtask 3:
void measureTask(void *args) {
	TickType_t xLastWakeTime;
	BaseType_t xStatus;
	xLastWakeTime = xTaskGetTickCount();
	queue_data_t data;

	for (;;) {
		data.measurement = measurement;
		data.counter = counter++;
		xStatus = xQueueOverwrite(queue, &data);	// nadpisz wartosc do kolejki

		if (xStatus != pdPASS) {	// nie udalo sie
			if (xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE) {
				queueError = QueueWriteProblem;
				xSemaphoreGive(mutex);
			}
		}

		if (xStatus == pdPASS) {		// udalo sie
			if (xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE) {
				queueError = QueueOK;
				xSemaphoreGive(mutex);
			}
		}

		vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1000));	// 1000 ms period
	}
}

void commTask(void *args) {
	TickType_t xLastWakeTime;
	queue_data_t data;
	uint32_t last_measurement_id = 0;
	uint16_t period = 400;
	uint8_t new_data_flag = 0; // 0 = no new data ; 1 = new data available
	uint8_t error_flag = 1; // 0 = fine ; 1 = queue is empty
	BaseType_t queue_size;

	xLastWakeTime = xTaskGetTickCount();
	queue_size = uxQueueMessagesWaiting(queue);

	for (;;) {
		if (xQueuePeek(queue, &data, pdMS_TO_TICKS(100)) == pdPASS) {	// podgladamy wartosc z kolejki
			if (last_measurement_id != data.counter) {
				new_data_flag = 1; 			// there is new data
				error_flag = 0;
				last_measurement_id = data.counter;	// update local counter
			} else if (queue_size == 0) {
				new_data_flag = 0;
				error_flag = 1;
			} else {
				new_data_flag = 0; 		// if counters are the same, then there is no new data
				error_flag = 0;
			}
		} else {
			new_data_flag = 0;
			error_flag = 1;		// failure in xQueuePeek
		}

		printf("259382; %lu; %u; %lu; %d; %d\r\n", HAL_GetTick(), data.measurement, data.counter, new_data_flag, error_flag);

		vTaskDelayUntil(&xLastWakeTime, period);
	}
}



// subtask 2:
//void measureTask(void *args) {
//	TickType_t xLastWakeTime;
//	BaseType_t xStatus;
//
//	xLastWakeTime = xTaskGetTickCount();
//
//	for (;;) {
//		xStatus = xQueueSendToBack(queue, &measurement, 0);	// wstaw wartosc do kolejki
//
//		if (xStatus != pdPASS) {	// nie udalo sie
//			if (xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE) {
//				queueError = QueueWriteProblem;
//				xSemaphoreGive(mutex);
//			}
//		}
//
//		if (xStatus == pdPASS) {		// udalo sie
//			if (xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE) {
//				queueError = QueueOK;
//				xSemaphoreGive(mutex);
//			}
//		}
//
//		vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(300));	// 300 ms period
//	}
//}
//
//void commTask(void *args) {
//	TickType_t xLastWakeTime;
//	uint16_t measurement_local = 0;
//	uint16_t flag_local;
//	uint16_t histeresis = 0;
//	BaseType_t queue_size;
//	BaseType_t xStatus;
//
//	xLastWakeTime = xTaskGetTickCount();
//	TickType_t period = pdMS_TO_TICKS(300);	// normal speed: 300, fast: 100 (more than 12 samples in queue), slow: 500 (less than 4 samples in queue)
//
//	for (;;) {
//		queue_size = uxQueueMessagesWaiting(queue);		// rozmiar kolejki
//
//		if (queue_size > 12 && histeresis != 1) {		// dopasowanie okresu
//			period = pdMS_TO_TICKS(100);
//			histeresis = 1;
//		} else if (queue_size < 4 && histeresis != 2) {
//			period = pdMS_TO_TICKS(500);
//			histeresis = 2;
//		}
//
//		xStatus = xQueueReceive(queue, &measurement_local, 0);	// odczyt z kolejki do lokalnej zmiennej
//
//
//		if (xStatus == pdPASS) {		// ustaw lokalnie flage bledu
//			flag_local = QueueOK;
//		} else if (queue_size == 0) {
//			flag_local = QueueEmpty;
//		} else {
//			flag_local = QueueCantRead;
//		}
//
//		if (xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE) {	// ustaw flage globalnie
//			queueError = flag_local;
//			xSemaphoreGive(mutex);
//		}
//
//		if (xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE) {	// odczytaj status przed printfem
//			flag_local = queueError;
//			xSemaphoreGive(mutex);
//		}
//
////		printf("Time: %lu, measured value: %u, queue size: %lu, error: %d\r\n ", HAL_GetTick(), measurement_local, queue_size, flag_local);
//		printf("259382; %lu; %u; %lu; %d\r\n", HAL_GetTick(), measurement_local, queue_size, flag_local);
//
//		vTaskDelayUntil(&xLastWakeTime, period);
//	}
//}





/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_TIM6_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

	// --> start TIM1 to generate PWM signal on TIMER3 connector
  	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	// --> start TIM6 in interrupt
  	HAL_TIM_Base_Start_IT(&htim6);	// subtask 1: HAL_TIM_Base_Start(&htim6);
	// --> start ADC1 in DMA mode
  	HAL_ADC_Start_DMA(&hadc1, (uint32_t *) &measurement, 1);
	// --> create a mutex
  	mutex = xSemaphoreCreateMutex();
	// --> create a queue
  	queue = xQueueCreate(1, sizeof(queue_data_t));
	// --> create all necessary tasks
  	xTaskCreate(measureTask, "measure", configMINIMAL_STACK_SIZE * 4, NULL, 3, NULL);
  	xTaskCreate(commTask, "comm", configMINIMAL_STACK_SIZE * 4, NULL, 3, NULL);
	printf("Starting!\r\n");

	// --> start FreeRTOS scheduler
	vTaskStartScheduler();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
		// subtask 1:
		//printf("259382; %lu; %u\r\n", HAL_GetTick(), measurement);
		//HAL_Delay(1000);
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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_ADC;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_HSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 8;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_ADC1CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
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

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os2.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "draw.h"
#include "stdio.h"
#include "draw.h"
#include "ux_api.h"
#include "modbus.h"
#include "errno.h"

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
/* USER CODE BEGIN Variables */
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart2;

int Wait_UART2_Tx_Complete(int timeout);
int Wait_UART4_Rx_Complete(int timeout);
int UART4_GetData(uint8_t *pData);
void UART4_Rx_Start(void);

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

static void LibmodbusServerTask( void *pvParameters )	
{
	uint8_t *query;
	modbus_t *ctx;
	int rc;
	modbus_mapping_t *mb_mapping;
	
	ctx = modbus_new_st_rtu("usb", 115200, 'N', 8, 1);
	modbus_set_slave(ctx, 1);
	query = pvPortMalloc(MODBUS_RTU_MAX_ADU_LENGTH);

	mb_mapping = modbus_mapping_new_start_address(0,
												  10,
												  0,
												  10,
												  0,
												  10,
												  0,
												  10);
	
	memset(mb_mapping->tab_bits, 0, mb_mapping->nb_bits);
	memset(mb_mapping->tab_registers, 0x55, mb_mapping->nb_registers*2);

	rc = modbus_connect(ctx);
	if (rc == -1) {
		//fprintf(stderr, "Unable to connect %s\n", modbus_strerror(errno));
		modbus_free(ctx);
		vTaskDelete(NULL);;
	}

	for (;;) {
		do {
			rc = modbus_receive(ctx, query);
			/* Filtered queries return 0 */
		} while (rc == 0);
 
		/* The connection is not closed on errors which require on reply such as
		   bad CRC in RTU. */
		if (rc == -1 && errno != EMBBADCRC) {
			/* Quit */
			continue;
		}

		rc = modbus_reply(ctx, query, rc, mb_mapping);
		if (rc == -1) {
			//break;
		}

		if (mb_mapping->tab_bits[0])
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_RESET);
		else
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_SET);
	}

	modbus_mapping_free(mb_mapping);
	vPortFree(query);
	/* For RTU */
	modbus_close(ctx);
	modbus_free(ctx);

	vTaskDelete(NULL);
}


static void LibmodbusClientTask( void *pvParameters )	
{
	modbus_t *ctx;
	int rc;
	uint16_t val;
	int nb = 1;
	
	ctx = modbus_new_st_rtu("usb", 115200, 'N', 8, 1);
	modbus_set_slave(ctx, 1);
	
	rc = modbus_connect(ctx);
	if (rc == -1) {
		//fprintf(stderr, "Unable to connect %s\n", modbus_strerror(errno));
		modbus_free(ctx);
		vTaskDelete(NULL);;
	}

	for (;;) {
		/* read hoding register 1 */
		rc = modbus_read_registers(ctx, 1, nb, &val);
		if (rc != nb)
			continue;

		/* display on lcd */
		Draw_Number(0, 0, val, 0xff0000);

		/* val ++ */
		val++;

		/* write val to hoding register 2 */
		rc = modbus_write_registers(ctx, 2, nb, &val);
	}

	/* For RTU */
	modbus_close(ctx);
	modbus_free(ctx);

	vTaskDelete(NULL);
}


static void CH2_UART4_ServerTask( void *pvParameters )	
{
	uint8_t *query;
	modbus_t *ctx;
	int rc;
	modbus_mapping_t *mb_mapping;
	char buf[100];
	int cnt = 0;
	
	ctx = modbus_new_st_rtu("uart4", 115200, 'N', 8, 1);
	modbus_set_slave(ctx, 1);
	query = pvPortMalloc(MODBUS_RTU_MAX_ADU_LENGTH);

	mb_mapping = modbus_mapping_new_start_address(0,
												  10,
												  0,
												  10,
												  0,
												  10,
												  0,
												  10);
	
	memset(mb_mapping->tab_bits, 0, mb_mapping->nb_bits);
	memset(mb_mapping->tab_registers, 0x55, mb_mapping->nb_registers*2);

	rc = modbus_connect(ctx);
	if (rc == -1) {
		//fprintf(stderr, "Unable to connect %s\n", modbus_strerror(errno));
		modbus_free(ctx);
		vTaskDelete(NULL);;
	}

	for (;;) {
		do {
			rc = modbus_receive(ctx, query);
			/* Filtered queries return 0 */
		} while (rc == 0);
 
		/* The connection is not closed on errors which require on reply such as
		   bad CRC in RTU. */
		if (rc == -1 && errno != EMBBADCRC) {
			/* Quit */
			continue;
		}

		rc = modbus_reply(ctx, query, rc, mb_mapping);
		if (rc == -1) {
			//break;
		}

		if (mb_mapping->tab_bits[0])
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_RESET);
		else
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_SET);

		vTaskDelay(1000);
		mb_mapping->tab_registers[1]++;
	}

	modbus_mapping_free(mb_mapping);
	vPortFree(query);
	/* For RTU */
	modbus_close(ctx);
	modbus_free(ctx);

	vTaskDelete(NULL);
}


static void CH1_UART2_ClientTask( void *pvParameters )	
{
	modbus_t *ctx;
	int rc;
	uint16_t val;
	int nb = 1;
	int level = 1;
	char buf[100];
	int cnt = 0;
	
	ctx = modbus_new_st_rtu("uart2", 115200, 'N', 8, 1);
	modbus_set_slave(ctx, 1);
	
	rc = modbus_connect(ctx);
	if (rc == -1) {
		//fprintf(stderr, "Unable to connect %s\n", modbus_strerror(errno));
		modbus_free(ctx);
		vTaskDelete(NULL);;
	}

	for (;;) {
		/* read hoding register 1 */
		rc = modbus_read_registers(ctx, 1, nb, &val);
		if (rc != nb)
		{
			continue;
		}

		/* display on lcd */
		Draw_Number(0, 0, val, 0xff0000);

		/* delay 2s */
		vTaskDelay(2000);
		modbus_write_bit(ctx, 0, level);
		level = !level;
	}

	/* For RTU */
	modbus_close(ctx);
	modbus_free(ctx);

	vTaskDelete(NULL);
}

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
#if 0  
	xTaskCreate(
		LibmodbusServerTask, // 鍑芥暟鎸囬拡, 浠诲姟鍑芥暟
		"LibmodbusServerTask", // 浠诲姟鐨勫悕瀛?
		200, // 鏍堝ぇ灏?,鍗曚綅涓簑ord,10琛ㄧず40瀛楄妭
		NULL, // 璋冪敤浠诲姟鍑芥暟鏃朵紶鍏ョ殑鍙傛暟
		osPriorityNormal, // 浼樺厛绾?
		NULL); // 浠诲姟鍙ユ焺, 浠ュ悗浣跨敤瀹冩潵鎿嶄綔杩欎釜浠诲姟
//#else
	xTaskCreate(
		LibmodbusClientTask, // 鍑芥暟鎸囬拡, 浠诲姟鍑芥暟
		"LibmodbusClientTask", // 浠诲姟鐨勫悕瀛?
		200, // 鏍堝ぇ灏?,鍗曚綅涓簑ord,10琛ㄧず40瀛楄妭
		NULL, // 璋冪敤浠诲姟鍑芥暟鏃朵紶鍏ョ殑鍙傛暟
		osPriorityNormal, // 浼樺厛绾?
		NULL); // 浠诲姟鍙ユ焺, 浠ュ悗浣跨敤瀹冩潵鎿嶄綔杩欎釜浠诲姟
#endif

  xTaskCreate(
	  CH1_UART2_ClientTask, // 鍑芥暟鎸囬拡, 浠诲姟鍑芥暟
	  "CH1_UART2_ClientTask", // 浠诲姟鐨勫悕
	  200, // 鏍?
	  NULL, // 璋冪敤浠诲姟鍑芥暟鏃朵紶鍏ョ殑鍙傛暟
	  osPriorityNormal, // 浼樺厛绾?
	  NULL); // 浠诲姟鍙ユ焺, 浠ュ悗浣跨敤瀹冩潵鎿嶄綔杩欎釜浠诲姟

  xTaskCreate(
	  CH2_UART4_ServerTask, // 鍑芥暟鎸囬拡, 浠诲姟鍑芥暟
	  "CH2_UART4_ServerTask", // 浠诲姟鐨勫悕
	  200, // 鏍?
	  NULL, // 璋冪敤浠诲姟鍑芥暟鏃朵紶鍏ョ殑鍙傛暟
	  osPriorityNormal, // 浼樺厛绾?
	  NULL); // 浠诲姟鍙ユ焺, 浠ュ悗浣跨敤瀹冩潵鎿嶄綔杩欎釜浠诲姟

  /* USER CODE END RTOS_THREADS */
 
  /* USER CODE BEGIN RTOS_EVENTS */ 
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}
/* USER CODE BEGIN Header_StartDefaultTask */
/**
* @brief Function implementing the defaultTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN defaultTask */
  /* Infinite loop */
  for(;;)
  {
// 		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_RESET);
//		vTaskDelay(500);
//		
//		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_SET);
//		vTaskDelay(500);
	  ux_system_tasks_run();
  }
  /* USER CODE END defaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */


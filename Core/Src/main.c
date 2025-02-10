/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define STATE_TEXT_LEN 20
#define CMD_ECHO_LEN 10

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* Definitions for Read_Sensors */
osThreadId_t Read_SensorsHandle;
uint32_t Read_SensorsBuffer[ 128 ];
osStaticThreadDef_t Read_SensorsControlBlock;
const osThreadAttr_t Read_Sensors_attributes = {
  .name = "Read_Sensors",
  .stack_mem = &Read_SensorsBuffer[0],
  .stack_size = sizeof(Read_SensorsBuffer),
  .cb_mem = &Read_SensorsControlBlock,
  .cb_size = sizeof(Read_SensorsControlBlock),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Read_Commands */
osThreadId_t Read_CommandsHandle;
uint32_t Read_CommandsBuffer[ 128 ];
osStaticThreadDef_t Read_CommandsControlBlock;
const osThreadAttr_t Read_Commands_attributes = {
  .name = "Read_Commands",
  .stack_mem = &Read_CommandsBuffer[0],
  .stack_size = sizeof(Read_CommandsBuffer),
  .cb_mem = &Read_CommandsControlBlock,
  .cb_size = sizeof(Read_CommandsControlBlock),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Camera_Control */
osThreadId_t Camera_ControlHandle;
uint32_t Camera_ControlBuffer[ 128 ];
osStaticThreadDef_t Camera_ControlControlBlock;
const osThreadAttr_t Camera_Control_attributes = {
  .name = "Camera_Control",
  .stack_mem = &Camera_ControlBuffer[0],
  .stack_size = sizeof(Camera_ControlBuffer),
  .cb_mem = &Camera_ControlControlBlock,
  .cb_size = sizeof(Camera_ControlControlBlock),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Send_Telemetry */
osThreadId_t Send_TelemetryHandle;
uint32_t Send_TelemetryBuffer[ 128 ];
osStaticThreadDef_t Send_TelemetryControlBlock;
const osThreadAttr_t Send_Telemetry_attributes = {
  .name = "Send_Telemetry",
  .stack_mem = &Send_TelemetryBuffer[0],
  .stack_size = sizeof(Send_TelemetryBuffer),
  .cb_mem = &Send_TelemetryControlBlock,
  .cb_size = sizeof(Send_TelemetryControlBlock),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Altitude_Check */
osThreadId_t Altitude_CheckHandle;
uint32_t Altitude_CheckBuffer[ 128 ];
osStaticThreadDef_t Altitude_CheckControlBlock;
const osThreadAttr_t Altitude_Check_attributes = {
  .name = "Altitude_Check",
  .stack_mem = &Altitude_CheckBuffer[0],
  .stack_size = sizeof(Altitude_CheckBuffer),
  .cb_mem = &Altitude_CheckControlBlock,
  .cb_size = sizeof(Altitude_CheckControlBlock),
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void ReadSensors(void *argument);
void ReadCommands(void *argument);
void CameraControl(void *argument);
void SendTelemetry(void *argument);
void AltitudeCheck(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//global flags
uint8_t transmit_enable = 0;
uint8_t simulation_enable = 0;

//structs

typedef struct telemetry_packet_data{
	int16_t  TEAM_ID;
	uint32_t MISSION_TIME; //store as seconds since midnight, decode later
	uint32_t PACKET_COUNT;
	char     MODE;
	char     STATE[STATE_TEXT_LEN];

	float    ALTITUDE;
	float    TEMPERATURE;
	float    PRESSURE;
	float 	 VOLTAGE; //might change to int, can multiply by 10 and store as int

	float    GYRO_R;
	float    GYRO_P;
	float    GYRO_Y;

	float    ACCEL_R;
	float    ACCEL_P;
	float    ACCEL_Y;

	float    MAG_R;
	float    MAG_P;
	float    MAG_Y;

	int16_t  AUTO_GYRO_ROTATION_RATE;

	uint32_t GPS_TIME;
	float    GPS_ALTITUDE;
	float    GPS_LATITUDE;
	float    GPS_LONGITUDE;
	uint8_t  GPS_SATS;

	char     CMD_ECHO[CMD_ECHO_LEN];

	//might add more as needed (optional)
};

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
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

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

  /* Create the thread(s) */
  /* creation of Read_Sensors */
  Read_SensorsHandle = osThreadNew(ReadSensors, NULL, &Read_Sensors_attributes);

  /* creation of Read_Commands */
  Read_CommandsHandle = osThreadNew(ReadCommands, NULL, &Read_Commands_attributes);

  /* creation of Camera_Control */
  Camera_ControlHandle = osThreadNew(CameraControl, NULL, &Camera_Control_attributes);

  /* creation of Send_Telemetry */
  Send_TelemetryHandle = osThreadNew(SendTelemetry, NULL, &Send_Telemetry_attributes);

  /* creation of Altitude_Check */
  Altitude_CheckHandle = osThreadNew(AltitudeCheck, NULL, &Altitude_Check_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_ReadSensors */
/**
  * @brief  Function implementing the Read_Sensors thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_ReadSensors */
void ReadSensors(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
	TickType_t xLastWakeTime = xTaskGetTickCount();
	const TickType_t xFrequency = pdMS_TO_TICKS(100);  // 100ms period for 10Hz

  for(;;)
  {

	  //Transmission_Control_s
	  vTaskDelayUntil(&xLastWakeTime, xFrequency);



    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_ReadCommands */
/**
* @brief Function implementing the Read_Commands thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ReadCommands */
void ReadCommands(void *argument)
{
  /* USER CODE BEGIN ReadCommands */
  /* Infinite loop */
  for(;;)
  {


	  char incoming[] = "whatever";

    osDelay(1);
  }
  /* USER CODE END ReadCommands */
}

/* USER CODE BEGIN Header_CameraControl */
/**
* @brief Function implementing the Camera_Control thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_CameraControl */
void CameraControl(void *argument)
{
  /* USER CODE BEGIN CameraControl */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END CameraControl */
}

/* USER CODE BEGIN Header_SendTelemetry */
/**
* @brief Function implementing the Send_Telemetry thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_SendTelemetry */
void SendTelemetry(void *argument)
{
  /* USER CODE BEGIN SendTelemetry */
  /* Infinite loop */
	TickType_t xLastWakeTime = xTaskGetTickCount();
	const TickType_t xFrequency = pdMS_TO_TICKS(100);  // 100ms period for 10Hz
  for(;;)
  {
	  //osSemaphoreWait(Transmission_ControlHandle, 0); //0 in case of no time out

	  vTaskDelayUntil(&xLastWakeTime, xFrequency);

	  //do something


  }
  /* USER CODE END SendTelemetry */
}

/* USER CODE BEGIN Header_AltitudeCheck */
/**
* @brief Function implementing the Altitude_Check thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_AltitudeCheck */
void AltitudeCheck(void *argument)
{
  /* USER CODE BEGIN AltitudeCheck */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END AltitudeCheck */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM7 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM7) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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

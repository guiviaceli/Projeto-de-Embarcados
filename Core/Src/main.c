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
#include "cmsis_os.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"
#include "FreeRTOS_CLI.h"
#include "usbd_cdc_if.h"
#include "arm_const_structs.h"
#include "arm_common_tables.h"
#include "arm_math.h"
#include "stdio.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX_INPUT_LENGTH    256
#define MAX_OUTPUT_LENGTH   256
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint32_t is_usb_on(void);
uint8_t adcBuffer[256];
float ReIm[256*2];
float mod[256];

typedef struct led_t_ {
	GPIO_TypeDef* port;
	uint16_t pin;
	int timeout;
}led_t;

led_t green_led;
led_t red_led;

void task_led(void *param)
{
	led_t *led = (led_t *)param;
	while(1)
	{
		HAL_GPIO_TogglePin(led->port, led->pin);
		vTaskDelay(led->timeout);
	}
}

void task_adc(void *param){
	HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adcBuffer, 256);
	HAL_TIM_Base_Start(&htim2);
	while(1){
		int k = 0;
		for(int i = 0; i < 256; i++){
			ReIm[k] = (float) adcBuffer[i] * 0.0007326007;
			ReIm[k+1] = 0.0;
			k += 2;
		}

		arm_cfft_f32(&arm_cfft_sR_f32_len256,ReIm,0,1);
		arm_cmplx_mag_f32(ReIm,mod,256);
		arm_scale_f32(mod, 0.0078125, mod, 128); /* vertor, por quem quero multiplicar, vetor final, quantos pontos */

		volatile float fund_phase = atan2f(ReIm[3],ReIm[2])*180/M_PI;
		(void)fund_phase;
		vTaskDelay(5);
	}
}

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
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
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
/**
* @}
*/
/**
* @}
*/

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
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  green_led.timeout = 500;
  green_led.port = GPIOG;
  green_led.pin = GPIO_PIN_13;

  red_led.timeout = 250;
  red_led.port = GPIOG;
  red_led.pin = GPIO_PIN_14;

  xTaskCreate(task_led, "Tarefa LED VERDE", 256, &green_led, 1, NULL);
  xTaskCreate(task_led, "Tarefa LED VERMELHO", 256, &red_led, 2, NULL);
  xTaskCreate(task_adc, "Tarefa ADC", 256, NULL, 5, NULL);
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;

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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 949;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 20;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV4;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 50000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 32700;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 20000;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13|GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pins : PG13 PG14 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
//uint8_t buffer[128];
uint32_t len;

uint8_t read_usb_cdc(char *buffer, int buf_len, TickType_t timeout);

static BaseType_t prvTaskStatsCommand( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString ){


        /* For simplicity, this function assumes the output buffer is large enough
        to hold all the text generated by executing the vTaskList() API function,
        so the xWriteBufferLen parameter is not used. */
        char *head = "Name\t\t\t\tState  Priority  Stack  Number\n\r";
        ( void ) xWriteBufferLen;

        /* pcWriteBuffer is used directly as the vTaskList() parameter, so the table
        generated by executing vTaskList() is written directly into the output
        buffer. */
        strcpy(pcWriteBuffer, head);
        vTaskList( pcWriteBuffer + strlen(head));

        /* The entire table was written directly to the output buffer.  Execution
        of this command is complete, so return pdFALSE. */
        return pdFALSE;
}
/*
static BaseType_t prvTaskStatsTexto( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString ){

	strcpy(pcWriteBuffer,(char*)"Este e um texto teste\r\n");
	return pdFALSE;
}*/

static BaseType_t prvTaskStatsHarmonica( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString ){
	const char *pcParameter1;
	const char *pcParameterAux;
	BaseType_t xParameter1StringLength;
	char comando1[5];
	char restorno[6];
	uint8_t valor1,valor2;
	comando1[0] = '\0';
	pcParameter1 = FreeRTOS_CLIGetParameter
	                        (
	                          /* The command string itself. */
	                          pcCommandString,
	                          /* Return the first parameter. */
	                          1,
	                          /* Store the parameter string length. */
	                          &xParameter1StringLength
	                        );

	strncpy(comando1,pcParameter1,xParameter1StringLength);
	comando1[xParameter1StringLength] = '\0';
	if(strcmp(comando1,(const char *)"cc") == (int)0){
		strcpy(pcWriteBuffer,(char*)"Este e o nivel CC do sinal: ");
		float auxf;
		auxf = (mod[0]*0.5);
		valor1 = (uint8_t) auxf;
		valor2 = (uint8_t) ((auxf-valor1)*10);
		restorno[0] = 48+valor1;
		restorno[1] = '.';
		restorno[2] = 48+valor2;
		valor2 = (uint8_t) ((((auxf-valor1)*10)-valor2)*10 );
		restorno[3] = 48+valor2;
		restorno[4] = '\0';
		strcpy(pcWriteBuffer + strlen(pcWriteBuffer),(char*)restorno);
	} else {
		uint8_t harm;
		harm = atoi(comando1);
		if(harm > 0 && harm < 255){
			uint8_t i, qnt;
			strcpy(pcWriteBuffer,(char*)"Este e a hamornica iniciada com ");
			strcpy(pcWriteBuffer + strlen(pcWriteBuffer),comando1);
			strcpy(pcWriteBuffer + strlen(pcWriteBuffer),(char*)": ");
			comando1[0] = '\0';
			pcParameterAux = FreeRTOS_CLIGetParameter
				                        (
				                          /* The command string itself. */
				                          pcCommandString,
				                          /* Return the first parameter. */
				                          2,
				                          /* Store the parameter string length. */
				                          &xParameter1StringLength
				                        );

			strncpy(comando1,pcParameterAux,xParameter1StringLength);
			comando1[xParameter1StringLength] = '\0';
			qnt = atoi(pcParameterAux);
			if(qnt > 0 && qnt < 128){
				float auxf;
				for(i = harm; i < (harm+qnt);i++){
					auxf = (mod[i]*0.5);
					valor1 = (uint8_t) auxf;
					valor2 = (uint8_t) ((auxf-valor1)*10);
					restorno[0] = 48+valor1;
					restorno[1] = '.';
					restorno[2] = 48+valor2;
					valor2 = (uint8_t) ((((auxf-valor1)*10)-valor2)*10 );
					restorno[3] = 48+valor2;
					if(i < (harm+qnt-1)){
						restorno[4] = ',';
						restorno[5] = ' ';
					} else {
						restorno[4] = '\0';
					}
					strcpy(pcWriteBuffer + strlen(pcWriteBuffer),(char*)restorno);
				}
			}
		}
	}
	return pdFALSE;
}

static const CLI_Command_Definition_t xTasksCommand =
{
    "tasks",
	"\r\ntasks:\r\n Lists all the installed tasks\r\n\r\n",
	prvTaskStatsCommand,
    0
};
/*
static const CLI_Command_Definition_t xTasksTexto =
{
    "texto",
	"\r\ntexto:\r\n Print Text Teste\r\n\r\n",
	prvTaskStatsTexto,
    0
};*/

static const CLI_Command_Definition_t xTasksHarmonica =
{
    "harmonica",
	"\r\nharmonica:\r\n fundamental, quantidade sequencial\r\n\r\n"
	"\r\nharmonica:\r\n cc, primeira fundamental \r\n\r\n",
	prvTaskStatsHarmonica,
    -1
};
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 5 */
  init_usb_rtos_obj();

  FreeRTOS_CLIRegisterCommand( &xTasksCommand );
  FreeRTOS_CLIRegisterCommand( &xTasksHarmonica );

  char data[128];
  /* Infinite loop */
  for(;;)
  {
	  queue_print(data,1);
  }
  /* USER CODE END 5 */
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

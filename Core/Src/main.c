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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
  MOTOR_IDLE,
  MOTOR_RUNNING,
  MOTOR_STOPPED
} MotorState;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Communication and buffer definitions
#define RX_BUFFER_SIZE 50       // Buffer size for UART reception
#define BUFFER_SIZE 1           // General buffer size
#define NUM_ADC_CHANNELS 1      // Number of ADC channels in use

// Motor control definitions
#define ALPHA 0.05            // Filter coefficient for EMA filter

// Motor parameters
#define PRESCALER 15 //COME BACK TO THIS
#define STEPS_PER 1600
#define MIN_SECONDS 60
#define CLOCK_FREQUENCY 16000000 // 16 MHz

#define COUNTS_PER_REV 2400.0f

// Torque Sensor Constants
#define VREF_ADC 3.3f
#define ADC_MAX  4095.0f
#define INA_GAIN 330
#define EXCITATION_VOLTAGE 5.0f
#define SENSOR_SENSTIVITY_MVV 1.5f
#define SENSOR_RATED_TORQUE 50.0f
#define ADC_AVG_WINDOW 16

// #define TORQUE_CAL_SLOPE 0.0008667f
#define TORQUE_CAL_SLOPE 0.000886088f


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
uint32_t flag=0;
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_tx;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */

volatile uint32_t adc_callback_count = 0;
volatile uint32_t stream_data_count = 0;

static uint32_t adc_avg_buf[ADC_AVG_WINDOW] = {0};
static uint8_t adc_avg_idx = 0;

uint32_t micros;

float_t strain_rate, strain, gauge_length, gauge_diameter; //Variables to be received from GUI

//Variables Used to Calculate Speed from Strain Rater and then to create PWM Signals
uint16_t prescaler = 16-1;
uint16_t steps_per = 1600;
uint16_t min_seconds = 60;
uint32_t clock_frequency = 16e6;
uint32_t num_samples = 100;
float_t duration, speed, sampling_interval;
volatile int16_t encoder_start_count = 0;
uint16_t calculatedsteps, ARR, frequency;
 
volatile uint16_t adc_buffer[NUM_ADC_CHANNELS];
volatile uint32_t tim2_overflow_count = 0;

//Variables for UART Communication
char msg[100];
uint8_t rx_buffer[RX_BUFFER_SIZE];
uint8_t rx_index = 0;

uint8_t received_char;

uint32_t arr1;

uint8_t packet[13];
 
volatile uint32_t sample_count = 0; 
volatile uint8_t recording_enabled = 0;
volatile uint8_t uart_dma_busy = 0;

volatile float_t filtered_value = 0.0f;
volatile float_t angle_degrees = 0.0f;
volatile uint16_t latest_torque_adc = 0;
 
volatile uint32_t target_steps = 0;
volatile uint32_t current_step = 0;
volatile uint8_t stepper_running = 0;
 
volatile MotorState motor_state = MOTOR_IDLE;

uint16_t adc_zero_offset; //ADC offset this is the value for which torque is 0

//Variable Changes when terminate test is clicked
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
void Stream_Data(void);
void Motor_Enable(uint8_t ena);
void Motor_SetPWM(uint32_t steps);
void Motor_SetDirection(uint8_t dir);
void Motor_RampUp(uint16_t maxFrequency, uint16_t steps);
float Apply_EMA_filter(float new_angle);
void Reset_Experiment_State(void);

uint16_t Tare_Torque_Sensor(void);

uint32_t GetMicroseconds(void); 

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
 
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
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_TIM1_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart2, &rx_buffer[rx_index], 1);
  HAL_ADCEx_Calibration_Start(&hadc1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    switch (motor_state)
    {
    case MOTOR_IDLE:
    if (strain_rate > 0)
    {
      Motor_Enable(1);
      if (strain_rate <= 13)
      {
        Motor_SetPWM(calculatedsteps);
        }
        else
        {
        //  Motor_RampUp(frequency, calculatedsteps);
        }
        motor_state = MOTOR_RUNNING;
    }
    break;
 
    case MOTOR_RUNNING:
      if (!stepper_running){
        motor_state = MOTOR_STOPPED;
      }
      // else if (!uart_dma_busy)
      // {
      //     Stream_Data();
      // }
      break;

    case MOTOR_STOPPED:
      Motor_Enable(0);
      Reset_Experiment_State();  // 🛠 full reset
      HAL_UART_AbortReceive_IT(&huart2);
      rx_index = 0;
      HAL_UART_Receive_IT(&huart2, &rx_buffer[rx_index], 1);
      break;
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_CC1;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 16-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC1;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */
 
  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 15;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_DISABLE;
  sSlaveConfig.InputTrigger = TIM_TS_ITR1;
  if (HAL_TIM_SlaveConfigSynchro(&htim2, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
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
  htim3.Init.Prescaler = 16-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */
 
  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 10;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */
 
  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */
 
  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 230400;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */
 
  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|DIR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ENA_Pin_GPIO_Port, ENA_Pin_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin DIR_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ENA_Pin_Pin */
  GPIO_InitStruct.Pin = ENA_Pin_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ENA_Pin_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/**
  * @brief  Applies an Exponential Moving Average (EMA) filter to smooth angle readings.
  * @param  new_angle: The latest angle value in degrees.
  * @retval Filtered angle value.
  * @note   The EMA filter reduces noise by blending new data with previous values.
  *         The smoothing factor is defined by ALPHA (0 < ALPHA < 1).
  *         A higher ALPHA gives more weight to new values, reducing lag but increasing noise.
  */
float Apply_EMA_filter(float float_value){
  if (filtered_value==0.0)
  {
    filtered_value= float_value;
  }
  else
  {
    filtered_value= (ALPHA*float_value)+((1-ALPHA)*filtered_value);
  }
  return filtered_value;
}

void Reset_Experiment_State(void) {
    stepper_running = 0;
    recording_enabled = 0;
    uart_dma_busy = 0;
    adc_callback_count = 0;
    stream_data_count = 0;
    sample_count = 0;
    tim2_overflow_count = 0;
    filtered_value = 0.0f;
    angle_degrees = 0.0f;

    // Stop peripherals safely
    HAL_TIM_PWM_Stop_IT(&htim3, TIM_CHANNEL_1);
    HAL_TIM_OC_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIM_Base_Stop(&htim1);
    HAL_TIM_Base_Stop(&htim2);
    HAL_TIM_Encoder_Stop(&htim4, TIM_CHANNEL_ALL);

    // Reset counters
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    __HAL_TIM_SET_COUNTER(&htim1, 0);
    __HAL_TIM_SET_COUNTER(&htim3, 0);
    __HAL_TIM_SET_COUNTER(&htim4, 0);

    __HAL_TIM_SET_COUNTER(&htim2, 0);
    tim2_overflow_count = 0;

    HAL_GPIO_WritePin(ENA_Pin_GPIO_Port, ENA_Pin_Pin, GPIO_PIN_RESET);
    motor_state = MOTOR_IDLE;
}


/**
 *
 */
void Motor_Steps(uint32_t steps, uint8_t direction)
{
  if (stepper_running) return;
 
  target_steps= steps;
  current_step = 0;
  stepper_running = 1;
 
  Motor_SetDirection(direction);
  __HAL_TIM_SET_COUNTER(&htim3, 0);
 
  HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_1);
}
 
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM3 && stepper_running) {
    current_step++;
    if (current_step >= target_steps) {
      HAL_TIM_PWM_Stop_IT(&htim3, TIM_CHANNEL_1);
      stepper_running = 0;
      motor_state = MOTOR_STOPPED;
      Motor_Enable(0);
      strain_rate = 0;
      recording_enabled = 0;

      Reset_Experiment_State();
      HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);

      uint8_t end_marker = 0xFF;
      HAL_UART_Transmit(&huart2, &end_marker, 1, HAL_MAX_DELAY);
    }
  }
}

 
/**
  * @brief Motor Enable Function- To Enable the Running of the Motor
  * @param ena: 0/1 value to turn the motor on or off
  * @retval None
  */
void Motor_Enable(uint8_t ena)
{
  if (ena)
  {
    HAL_GPIO_WritePin(ENA_Pin_GPIO_Port, ENA_Pin_Pin, GPIO_PIN_SET);
    recording_enabled = 1;

    filtered_value = 0.0f;
    angle_degrees = 0.0f;

    // 🔧 Reset TIM2 (timestamp) and counters
    tim2_overflow_count = 0;
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    __HAL_TIM_SET_COUNTER(&htim4, 0);
    sample_count = 0;

    __HAL_TIM_ENABLE_IT(&htim2, TIM_IT_UPDATE);  // TIM2 overflow interrupt (timestamping)
    HAL_TIM_Base_Start_IT(&htim2);                  // Start TIM2 (free-running time base)

    // arr1= 1000;
    

    // 🟢 TIM1 setup for sampling trigger (ADC, encoder, etc.)
    __HAL_TIM_SET_AUTORELOAD(&htim1, arr1);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, arr1);
    __HAL_TIM_SET_COUNTER(&htim1, 0);

    adc_zero_offset = Tare_Torque_Sensor();
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, NUM_ADC_CHANNELS);

        
    HAL_TIM_Base_Start_IT(&htim1);         // Start TIM1 counter
    HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_1);  // Enable output compare (triggers ADC)

    HAL_TIM_Encoder_Stop(&htim4, TIM_CHANNEL_ALL);
    __HAL_TIM_SET_COUNTER(&htim4, 0);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

    encoder_start_count = 0;   // zero reference
  }
  else
  {
    HAL_GPIO_WritePin(ENA_Pin_GPIO_Port, ENA_Pin_Pin, GPIO_PIN_RESET);

    HAL_TIM_PWM_Stop_IT(&htim3, TIM_CHANNEL_1);
    HAL_TIM_OC_Stop(&htim1, TIM_CHANNEL_1);       // <-- 🔧 Don't forget to stop OC channel
    HAL_TIM_Base_Stop(&htim1);
    HAL_TIM_Base_Stop(&htim2);

    HAL_TIM_Encoder_Stop(&htim4, TIM_CHANNEL_ALL);

    recording_enabled = 0;
    uart_dma_busy = 0;
  }
}

 
/**
  * @brief Motor Direction Function- Setting the Direction of Rotation
  * @param dir: 0/1 value to turn the motor CCW/CW
  * @retval None
  */
void Motor_SetDirection(uint8_t dir)
{
  if (dir)
  {
    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
  }
  else
  {
    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);
  }
}
 
/**
  * @brief Motor Step Function - In the Case the Speed of the Motor doesn't need an acceleration profile directly setting the PWM
  * @param None
  * @retval None
  */
 void Motor_SetPWM(uint32_t steps)
 {
  Motor_Steps(steps, 0);
 }
 
/**
  * @brief Timers Interrupt Function - To set up for Timer 4 (free-running timer), and Timer 2 (interrupt timer to get Timer 4 and Encoder Readings)
  * @param htim: Timer Instance
  * @retval None
  */
 void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM2) {
    tim2_overflow_count++;
    if (recording_enabled && motor_state == MOTOR_RUNNING) {
      sample_count++;
    }
  }
  else if (htim->Instance == TIM1){
    if (recording_enabled && motor_state == MOTOR_RUNNING){
    }
  }
}
 
float Calculate_Rotation(uint16_t steps) {
  return ((float_t)steps/(float_t)steps_per)*360.0;
}
 
float Calculate_Torque (uint16_t adc){
  // float v_adc = ((float)((int32_t)adc - (int32_t)adc_zero_offset) /  ADC_MAX) * VREF_ADC;
  // float v_bridge = v_adc/INA_GAIN;

  // float senstivity_v = (SENSOR_SENSTIVITY_MVV/1000.0f) * EXCITATION_VOLTAGE;

  // float torque_nm = (v_bridge/ senstivity_v) * SENSOR_RATED_TORQUE;
  float torque_nm = TORQUE_CAL_SLOPE * ((float)(int32_t)adc - (float)(int32_t)adc_zero_offset);

  return torque_nm;
}
 
/**
  * @brief  UART receive complete callback for USART2.
  * @param  huart: Pointer to the UART handle.
  * @retval None
  * @note   - Handles incoming UART messages.
  *         - If 'S' is received, stops the motor.
  *         - Parses incoming data to extract strain rate, strain, gauge length, and gauge diameter.
  *         - Calculates motor speed, frequency, and duration.
  *         - Starts motor operation based on received parameters.
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        received_char = rx_buffer[rx_index];
 
        if (received_char == 'S')
        {
            strain_rate = 0;
            Motor_Enable(0);
            motor_state = MOTOR_STOPPED;
            
            stepper_running = 0;
            recording_enabled = 0;
            uart_dma_busy= 0;
             
            memset(rx_buffer, 0, RX_BUFFER_SIZE);
            HAL_UART_AbortReceive_IT(&huart2);
            rx_index = 0;
            strain = 0;
            gauge_length = 0;
            gauge_diameter = 0;
            Reset_Experiment_State();

            HAL_UART_Receive_IT(&huart2, &rx_buffer[rx_index], 1);

            return;
        }
        else if (received_char == '\n' || rx_index >= RX_BUFFER_SIZE-1)
        {
            rx_buffer[rx_index] = '\0'; // Null terminate the string
            char *token = strtok((char *)rx_buffer, " ");
            if (token != NULL)
            {
                strain_rate = strtof(token, NULL);
                token = strtok(NULL, " ");
            }
            if (token != NULL)
            {
                strain = strtof(token, NULL);
                token = strtok(NULL, " ");
            }
            if (token != NULL)
            {
                gauge_length = strtof(token, NULL);
                token = strtok(NULL, " ");
            }
            if (token != NULL)
            {
                gauge_diameter = strtof(token, NULL);
            }
 
            speed = (sqrt(3) * strain_rate * gauge_length * min_seconds) / (M_PI * gauge_diameter);
            frequency = (speed * steps_per) / min_seconds;
            duration = strain / strain_rate;
            sampling_interval = duration/(float_t)num_samples;
            float_t arr1_float = sampling_interval * (clock_frequency / (prescaler + 1));
            arr1= ((uint32_t)arr1_float)-1;

            __HAL_TIM_SET_AUTORELOAD(&htim1, arr1);
            __HAL_TIM_SET_COUNTER(&htim1, 0);
 
            calculatedsteps = (strain * sqrt(3) * gauge_length*steps_per)/(gauge_diameter*M_PI);
 
            sprintf(msg, "Speed: %f, Frequency: %d, Duration: %f\r\n", speed, frequency, duration);
 
            ARR = clock_frequency / ((prescaler+1) * frequency);
            __HAL_TIM_SET_AUTORELOAD(&htim3, ARR - 1);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, ARR / 2);       
             memset(rx_buffer, 0, RX_BUFFER_SIZE);  // Clear buffer
            rx_index = 0;
        }
        else
        {
          rx_index++;
          if (rx_index >= RX_BUFFER_SIZE) {
               rx_index = 0;
          }
        }
    }
    HAL_UART_Receive_IT(&huart2, &rx_buffer[rx_index], 1);
}

/**
  * @brief  UART transmit complete callback for USART2.
  * @param  huart: Pointer to the UART handle.
  * @retval None
  * @note   - Handles completion of DMA-based UART transmission.
  *         - Continues sending the next chunk of stored data if available.
  *         - Resets transmission state when all data has been sent.
  */
 void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART2) {
      uart_dma_busy = 0;
  }
}

uint32_t GetMicroseconds(void) {
  uint32_t overflow, counter;

  __disable_irq();  // Prevent TIM2 overflow from happening mid-read
  overflow = tim2_overflow_count;
  counter = __HAL_TIM_GET_COUNTER(&htim2);
  if (__HAL_TIM_GET_FLAG(&htim2, TIM_FLAG_UPDATE)) {
      // Overflow occurred just after we read, but before ISR ran
      overflow++;
      counter = __HAL_TIM_GET_COUNTER(&htim2);  // Read again for accuracy
  }
  __enable_irq();
  return (overflow * 65536UL) + counter;
}

uint16_t Tare_Torque_Sensor(void){
  uint32_t sum = 0;
  const int N = 500;
  for (int i = 0; i < N; i++){
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 10);
    sum += HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);
  }
  return (uint16_t)(sum/N);
}

 
void Stream_Data(void) {
  if (!uart_dma_busy && recording_enabled && motor_state == MOTOR_RUNNING && stepper_running) {
    uart_dma_busy = 1;

    micros = GetMicroseconds();
    float tstamp = micros / 1e6f;
    float torque = Calculate_Torque(latest_torque_adc);
    float torque_filtered = Apply_EMA_filter(torque);
    // float torque_filtered = Apply_EMA_filter();

    // Read encoder as signed 16-bit to handle direction + wrap
    int16_t encoder_count = (int16_t)__HAL_TIM_GET_COUNTER(&htim4);

    // Compute delta from start position
    int16_t delta = encoder_count - encoder_start_count;

    // Convert to mechanical degrees
    float rotation_degrees = ((float)delta / COUNTS_PER_REV) * 360.0f;


    packet[0] = 0xAA;
    memcpy(&packet[1], &tstamp, sizeof(float));
    memcpy(&packet[5], &rotation_degrees, sizeof(float));
    memcpy(&packet[9], &torque_filtered, sizeof(float));

    HAL_UART_Transmit_IT(&huart2, packet, sizeof(packet));
    stream_data_count++;
  }
  else{
    return;
  }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
  if (hadc->Instance == ADC1 && recording_enabled && motor_state == MOTOR_RUNNING) {
    adc_avg_buf[adc_avg_idx] = adc_buffer[0];
    adc_avg_idx = (adc_avg_idx + 1) % ADC_AVG_WINDOW;

    uint32_t sum = 0;
    for (int i = 0; i < ADC_AVG_WINDOW; i++) sum += adc_avg_buf[i];

    latest_torque_adc = (uint16_t)(sum/ADC_AVG_WINDOW);
    adc_callback_count++;
    
    latest_torque_adc = adc_buffer[0];
    Stream_Data();
    adc_callback_count++;
  }
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

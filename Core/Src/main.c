/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body - UAV Flight Controller (STM32F411)
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mpu6050.h"
#include "bmp280.h"
#include "servo.h"
#include "pid.h"
#include "sbus.h"
#include "mav_telemetry.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define USE_MAVLINK         1       // 1 = aktifkan telemetri MAVLink ke Mission Planner, 0 = matikan
#define MAX_STAB_ANGLE      30.0f   // Batas sudut target saat mode Stabilize PID (derajat)
#define MAX_MANUAL_DEFLECT  45.0f   // Batas defleksi servo saat mode Manual (derajat)
#define TEST_SENSOR_ONLY    0       // 0 = mode terbang normal (Manual <-> Stabilize via SBUS), 1 = tes sensor saja
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
MPU6050_t mpu;
BMP280_t  bmp;
SBUS_t    sbus;

uint8_t   bmp_ok = 0;

// Objek aktuator permukaan kendali (Dual Aileron & Elevator)
Servo_t servo_aileron_right;  // TIM2 CH1 (PA0)
Servo_t servo_aileron_left;   // TIM2 CH2 (PA1)
Servo_t servo_elevator;       // TIM2 CH3 (PA2 / PB10)

// Objek PID untuk kestabilan Roll (Aileron) dan Pitch (Elevator)
PID_t pid_roll;
PID_t pid_pitch;

float roll_cmd  = 0.0f;
float pitch_cmd = 0.0f;

uint32_t last_tick = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

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
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_USB_DEVICE_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  // Jeda 100 ms agar sensor selesai power-up saat baterai dicolok
  HAL_Delay(100);

  // 1. Inisialisasi Servo Sesuai Konfigurasi Fisik
  Servo_Init(&servo_aileron_right, &htim2, TIM_CHANNEL_1); // Aileron Kanan
  Servo_Init(&servo_aileron_left,  &htim2, TIM_CHANNEL_2); // Aileron Kiri
  Servo_Init(&servo_elevator,      &htim2, TIM_CHANNEL_3); // Elevator

  // 2. Inisialisasi Sensor MPU6050 & Kalibrasi Gyro (Pastikan pesawat diam datar saat start!)
  if (MPU6050_Init(&hi2c1, &mpu) == HAL_OK) {
      MPU6050_Calibrate_Gyro(&hi2c1, &mpu, 500);
  }

  // 3. Inisialisasi Barometer & Ambil Baseline Altitude (Ground Level)
  if (BMP280_Init(&hi2c1, &bmp)) {
      bmp_ok = 1;
      BMP280_Calibrate_Altitude(&hi2c1, &bmp, 30);
  }

  // 4. Inisialisasi Receiver SBUS pada USART2 (PA3)
  SBUS_Init(&sbus, &huart2);

  // 5. Inisialisasi PID untuk Roll (Aileron) dan Pitch (Elevator)
  PID_Init(&pid_roll,  0.8f, 0.05f, 0.1f, -45.0f, 45.0f);
  PID_Init(&pid_pitch, 0.8f, 0.05f, 0.1f, -45.0f, 45.0f);

  // 6. Daftarkan parameter PID ke Mission Planner & aktifkan RX UART1 (57600 bps)
#if USE_MAVLINK
  mav_register_param("PID_ROLL_P",  &pid_roll.Kp);
  mav_register_param("PID_ROLL_I",  &pid_roll.Ki);
  mav_register_param("PID_ROLL_D",  &pid_roll.Kd);
  mav_register_param("PID_PITCH_P", &pid_pitch.Kp);
  mav_register_param("PID_PITCH_I", &pid_pitch.Ki);
  mav_register_param("PID_PITCH_D", &pid_pitch.Kd);

  #if MAV_OVER_UART
  mav_uart_start_rx();
  #endif
#endif

  last_tick = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    uint32_t current_tick = HAL_GetTick();
    float dt = (current_tick - last_tick) / 1000.0f;

    // Eksekusi kontrol loop setiap 10ms (100 Hz)
    if (dt >= 0.01f) {
        last_tick = current_tick;

        // A. Baca Sensor IMU & Hitung Sudut (Complementary Filter)
        if (MPU6050_Read_All(&hi2c1, &mpu) == HAL_OK) {
            MPU6050_Update_Angle(&mpu);
        }

        // B. Baca Barometer (Suhu, Tekanan, Ketinggian)
        if (bmp_ok) {
            BMP280_Update(&hi2c1, &bmp);
        }

        // C. Perhitungan Kendali (Mode Tes Sensor vs Mode Terbang Manual/Stabilize SBUS)
        roll_cmd  = 0.0f;
        pitch_cmd = 0.0f;

#if TEST_SENSOR_ONLY
        // Mode Uji Coba Sensor: PID langsung menstabilkan board ke sudut datar (0 derajat)
        roll_cmd  = PID_Compute(&pid_roll,  0.0f, mpu.roll,  dt);
        pitch_cmd = PID_Compute(&pid_pitch, 0.0f, mpu.pitch, dt);
#else
        if (SBUS_IsConnected(&sbus)) {
            // Channel 5 (indeks 4) sebagai sakelar mode terbang (> 1500 = Stabilize PID, <= 1500 = Manual)
            uint8_t mode_stabilize = (sbus.channels[4] > 1500) ? 1 : 0;

            if (mode_stabilize) {
                // Mode Stabilize: Stik menentukan target sudut (-30 s.d. +30 derajat), PID mengoreksi
                float roll_setpoint  = SBUS_ChannelToFloat(sbus.channels[0], -MAX_STAB_ANGLE, MAX_STAB_ANGLE);
                float pitch_setpoint = SBUS_ChannelToFloat(sbus.channels[1], -MAX_STAB_ANGLE, MAX_STAB_ANGLE);

                roll_cmd  = PID_Compute(&pid_roll,  roll_setpoint,  mpu.roll,  dt);
                pitch_cmd = PID_Compute(&pid_pitch, pitch_setpoint, mpu.pitch, dt);
            } else {
                // Mode Manual: Reset integral PID & gerakkan bidang kendali langsung (-45 s.d. +45 derajat)
                PID_Reset(&pid_roll);
                PID_Reset(&pid_pitch);

                roll_cmd  = SBUS_ChannelToFloat(sbus.channels[0], -MAX_MANUAL_DEFLECT, MAX_MANUAL_DEFLECT);
                pitch_cmd = SBUS_ChannelToFloat(sbus.channels[1], -MAX_MANUAL_DEFLECT, MAX_MANUAL_DEFLECT);
            }
        } else {
            // Failsafe: Sinyal receiver hilang -> netralkan bidang kendali (90 derajat)
            PID_Reset(&pid_roll);
            PID_Reset(&pid_pitch);
            roll_cmd  = 0.0f;
            pitch_cmd = 0.0f;
        }
#endif

        // D. Mixing ke Servo Aileron Kanan, Aileron Kiri, dan Elevator
        float aileron_right_angle = 90.0f + roll_cmd;
        float aileron_left_angle  = 90.0f - roll_cmd;
        float elevator_angle      = 90.0f + pitch_cmd;

        Servo_SetAngle(&servo_aileron_right, aileron_right_angle);
        Servo_SetAngle(&servo_aileron_left,  aileron_left_angle);
        Servo_SetAngle(&servo_elevator,      elevator_angle);

        // E. Kirim Telemetri MAVLink ke Mission Planner via USART1 (57600 bps)
#if USE_MAVLINK
        mav_telemetry_update(mpu.roll, mpu.pitch, roll_cmd, pitch_cmd, mpu.yaw, bmp.altitude);
#endif
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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  for (int i = 0; i < 9; i++) {
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
      HAL_Delay(1);
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
      HAL_Delay(1);
  }
  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 95;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 19999;
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
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  * @brief USART1 Initialization Function (Telemetry 57600 bps)
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 57600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  huart2.Init.BaudRate = 100000;
  huart2.Init.WordLength = UART_WORDLENGTH_9B;
  huart2.Init.StopBits = UART_STOPBITS_2;
  huart2.Init.Parity = UART_PARITY_EVEN;
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        SBUS_UART_RxCpltCallback(&sbus, huart);
    }
#if USE_MAVLINK && MAV_OVER_UART
    if (huart->Instance == USART1) {
        mav_uart_rx_callback(huart);
    }
#endif
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        __HAL_UART_CLEAR_OREFLAG(huart);
        __HAL_UART_CLEAR_FEFLAG(huart);
        __HAL_UART_CLEAR_NEFLAG(huart);
        HAL_UART_Receive_IT(huart, &sbus.rx_byte, 1);
    }
#if USE_MAVLINK && MAV_OVER_UART
    if (huart->Instance == USART1) {
        mav_uart_error_callback(huart);
    }
#endif
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
#ifdef USE_FULL_ASSERT
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

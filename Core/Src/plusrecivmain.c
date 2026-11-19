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
#include <string.h>
#include "usbd_cdc_if.h"

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
#define USE_MAVLINK         1       // 1 = aktifkan telemetri MAVLink ke PC, 0 = matikan
#define MAX_STAB_ANGLE      30.0f   // Batas sudut target saat mode Stabilize PID (derajat)
#define MAX_MANUAL_DEFLECT  45.0f   // Batas defleksi servo saat mode Manual (derajat)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
MPU6050_t mpu;
BMP280_t  bmp;
SBUS_t    sbus;

// Objek aktuator permukaan kendali (Dual Aileron & Elevator)
Servo_t servo_aileron_right;  // TIM2 CH1 (PA0)
Servo_t servo_aileron_left;   // TIM2 CH2 (PA1)
Servo_t servo_elevator;       // TIM2 CH3 (PA2)

// Objek PID untuk kestabilan Roll (Aileron) dan Pitch (Elevator)
PID_t pid_roll;
PID_t pid_pitch;

uint32_t last_tick = 0;
uint32_t last_mav_tick = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
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
  MX_USB_DEVICE_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  // 1. Inisialisasi Servo Sesuai Konfigurasi Fisik
  Servo_Init(&servo_aileron_right, &htim2, TIM_CHANNEL_1); // PA0 -> Aileron Kanan
  Servo_Init(&servo_aileron_left,  &htim2, TIM_CHANNEL_2); // PA1 -> Aileron Kiri
  Servo_Init(&servo_elevator,      &htim2, TIM_CHANNEL_3); // PA2 -> Elevator

  // 2. Inisialisasi Sensor MPU6050 & Kalibrasi Gyro
  if (MPU6050_Init(&hi2c1, &mpu) == HAL_OK) {
      MPU6050_Calibrate_Gyro(&hi2c1, &mpu, 500);
  }

  // 3. Inisialisasi BMP280 & Ambil Baseline Altitude (Ground Level)
  if (BMP280_Init(&hi2c1, &bmp)) {
      BMP280_Calibrate_Altitude(&hi2c1, &bmp, 50);
  }

  // 4. Inisialisasi Receiver SBUS pada USART1 (PA10)
  SBUS_Init(&sbus, &huart1);

  // 5. Inisialisasi PID untuk Roll (Aileron) dan Pitch (Elevator)
  PID_Init(&pid_roll,  1.2f, 0.05f, 0.1f, -45.0f, 45.0f);
  PID_Init(&pid_pitch, 1.2f, 0.05f, 0.1f, -45.0f, 45.0f);

  last_tick = HAL_GetTick();
  last_mav_tick = HAL_GetTick();
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
        BMP280_Read_All(&hi2c1, &bmp);

        // C. Pembacaan Perintah Receiver SBUS & Perhitungan Kendali
        float roll_cmd  = 0.0f;
        float pitch_cmd = 0.0f;

        if (SBUS_IsConnected(&sbus)) {
            // Channel 5 (indeks 4) sebagai sakelar mode terbang (> 1500 = Stabilize PID)
            uint8_t mode_stabilize = (sbus.channels[4] > 1500) ? 1 : 0;

            if (mode_stabilize) {
                // Mode Stabilize: Stik menentukan target sudut (-30° s.d. +30°), PID mengoreksi
                float roll_setpoint  = SBUS_ChannelToFloat(sbus.channels[0], -MAX_STAB_ANGLE, MAX_STAB_ANGLE);
                float pitch_setpoint = SBUS_ChannelToFloat(sbus.channels[1], -MAX_STAB_ANGLE, MAX_STAB_ANGLE);

                roll_cmd  = PID_Compute(&pid_roll,  roll_setpoint,  mpu.roll,  dt);
                pitch_cmd = PID_Compute(&pid_pitch, pitch_setpoint, mpu.pitch, dt);
            } else {
                // Mode Manual: Stik langsung menggerakkan bidang kendali (-45° s.d. +45°)
                PID_Reset(&pid_roll);
                PID_Reset(&pid_pitch);

                roll_cmd  = SBUS_ChannelToFloat(sbus.channels[0], -MAX_MANUAL_DEFLECT, MAX_MANUAL_DEFLECT);
                pitch_cmd = SBUS_ChannelToFloat(sbus.channels[1], -MAX_MANUAL_DEFLECT, MAX_MANUAL_DEFLECT);
            }
        } else {
            // Failsafe: Sinyal receiver hilang -> netralkan bidang kendali
            PID_Reset(&pid_roll);
            PID_Reset(&pid_pitch);
            roll_cmd  = 0.0f;
            pitch_cmd = 0.0f;
        }

        // D. Mixing ke Servo Aileron Kanan, Aileron Kiri, dan Elevator
        float aileron_right_angle = 90.0f + roll_cmd;
        float aileron_left_angle  = 90.0f - roll_cmd;
        float elevator_angle      = 90.0f + pitch_cmd;

        Servo_SetAngle(&servo_aileron_right, aileron_right_angle);
        Servo_SetAngle(&servo_aileron_left,  aileron_left_angle);
        Servo_SetAngle(&servo_elevator,      elevator_angle);
    }

#if USE_MAVLINK
    // E. Proses perintah tuning parameter QGC & kirim telemetri MAVLink 50 Hz
    mav_telemetry_update(mpu.roll, mpu.pitch, mpu.roll, mpu.pitch);
#endif
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
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 100 - 1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 20000 - 1;
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
  HAL_TIM_MspPostInit(&htim2);
}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

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
  HAL_TIM_MspPostInit(&htim3);
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 100000;
  huart1.Init.WordLength = UART_WORDLENGTH_9B;
  huart1.Init.StopBits = UART_STOPBITS_2;
  huart1.Init.Parity = UART_PARITY_EVEN;
  huart1.Init.Mode = UART_MODE_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    SBUS_UART_RxCpltCallback(&sbus, huart);
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  while (1)
  {
  }
}

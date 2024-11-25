/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MPU6050_ADDR1 (0x68 << 1)
#define MPU6050_ADDR2 (0x69 << 1)

#define FS_GYRO_250 0
#define FS_GYRO_500 8


#define FS_ACC_2G 0
#define FS_ACC_4G 8

// Other registers
#define REG_CONFIG_GYRO 27
#define REG_CONFIG_ACC 28
#define REG_USR_CTRL 107 // 0x6B is PWR_MGMT_1 register
#define REG_DATA 59

// ACCEL registers
#define ACCEL_XOUT_H 0x3B
#define ACCEL_XOUT_L 0x3C

#define ACCEL_YOUT_H 0x3D
#define ACCEL_YOUT_L 0x3E

#define ACCEL_ZOUT_H 0x3F
#define ACCEL_ZOUT_L 0x40
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Global variables for STM32cubeMonitor
HAL_StatusTypeDef ret;
uint8_t ret_ready = 10;
uint8_t ret_accel = 20;
uint8_t ret_exit_sleep = 20;

uint8_t alarm_status1 = 0;
uint8_t alarm_status2 = 0;

int16_t AccelX1, AccelY1, AccelZ1;
float AccelX1_g, AccelY1_g, AccelZ1_g;
int16_t AccelX2, AccelY2, AccelZ2;
float AccelX2_g, AccelY2_g, AccelZ2_g;

// Define the structure globally or before you use it
struct CalibrationData {
    float AccelX;
    float AccelY;
    float AccelZ;
};

// Initialize MPU
void MPU6050_Init(MPU6050_ADDR) {
  ret = HAL_I2C_IsDeviceReady(&hi2c1, MPU6050_ADDR, 1, 100);

  if (ret == HAL_OK) {
    ret_ready = 10;
  } else if (ret == HAL_ERROR) {
    ret_ready = 11;
    I2C_Bus_Recovery();
  } else if (ret == HAL_BUSY) {
    ret_ready = 12;
    I2C_Bus_Recovery();
  } else {
    ret_ready = 13;
    I2C_Bus_Recovery();
  }

  uint8_t temp_data = FS_ACC_4G;
  ret = HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, REG_CONFIG_ACC, 1, &temp_data, 1, HAL_MAX_DELAY);

  if (ret == HAL_OK) {
    ret_accel = 20;
  } else if (ret == HAL_ERROR) {
    ret_accel = 21;
    I2C_Bus_Recovery();
  } else if (ret == HAL_BUSY) {
    ret_accel = 22;
    I2C_Bus_Recovery();
  } else {
    ret_accel = 23;
    I2C_Bus_Recovery();
  }

  // Exit sleep by writing 0x00 to PWR_MGMT_1 register 0x6B
  uint8_t data = 0;
  ret = HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, REG_USR_CTRL, 1, &data, 1, HAL_MAX_DELAY);

  if (ret == HAL_OK) {
    ret_exit_sleep = 30;
  } else if (ret == HAL_ERROR) {
    ret_exit_sleep = 31;
  } else if (ret == HAL_BUSY) {
    ret_exit_sleep = 32;
  } else {
    ret_exit_sleep = 33;
  }
}

// Resolve HAL_BUSY error
void I2C_Bus_Recovery() {
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  // Reconfigure SCL and SDA as GPIO
  __HAL_RCC_GPIOB_CLK_ENABLE(); // Enable GPIO clock (adjust based on your pin)
  GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9; // Replace with your I2C pins
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  // Manually clock SCL line
  for (int i = 0; i < 21; i++) {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET); // SCL High
    HAL_Delay(20); // Short delay
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET); // SCL Low
    HAL_Delay(20);
  }

  // Reinitialize I2C
  HAL_I2C_DeInit(&hi2c1);
  HAL_I2C_Init(&hi2c1);
}

void MPU6050_Read_Accel(uint8_t MPU6050_ADDR, int16_t *AccelX, int16_t *AccelY, int16_t *AccelZ) {
  uint8_t buffer[6]; // Buffer to store the raw data (6 bytes for X, Y, Z)

  // Read 6 bytes starting from ACCEL_XOUT_H (0x3B)
  if (HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, ACCEL_XOUT_H, I2C_MEMADD_SIZE_8BIT, buffer, 6, HAL_MAX_DELAY) == HAL_OK) {
    // Combine high and low bytes for each axis
    *AccelX = (int16_t)(buffer[0] << 8 | buffer[1]); // X-axis
    *AccelY = (int16_t)(buffer[2] << 8 | buffer[3]); // Y-axis
    *AccelZ = (int16_t)(buffer[4] << 8 | buffer[5]); // Z-axis
  } else {
    // If reading fails, set default values or handle error
    I2C_Bus_Recovery();
    *AccelX = 0;
    *AccelY = 0;
    *AccelZ = 0;

  }
}

float MPU6050_Convert_to_g(int16_t raw_value) {
  return (float)raw_value / 16384.0;  // Assuming default ±2g sensitivity
}

struct CalibrationData MPU6050_Calibration(uint16_t dev_addr) {
  float AccelX_g = 0, AccelY_g = 0, AccelZ_g = 0;
  float AccelX_g_sum = 0, AccelY_g_sum = 0, AccelZ_g_sum = 0;
  int cnt = 0; // cnt should be an integer

  int16_t AccelX = 0, AccelY = 0, AccelZ = 0;

  // Read data 1024 times for calibration
  while (cnt < 1024) {
    // Read raw accelerometer data from the MPU6050 sensor
    MPU6050_Read_Accel(dev_addr, &AccelX, &AccelY, &AccelZ);

    // Convert the raw accelerometer data to 'g' units
    AccelX_g = MPU6050_Convert_to_g(AccelX);
    AccelY_g = MPU6050_Convert_to_g(AccelY);
    AccelZ_g = MPU6050_Convert_to_g(AccelZ);

    // Sum the 'g' values for averaging
    AccelX_g_sum += AccelX_g;
    AccelY_g_sum += AccelY_g;
    AccelZ_g_sum += AccelZ_g;

    cnt++; // increment the counter
  }

  // Average the sums
  AccelX_g_sum /= 1024;
  AccelY_g_sum /= 1024;
  AccelZ_g_sum /= 1024;

  // Create and return the CalibrationData structure
  struct CalibrationData data = {AccelX_g_sum, AccelY_g_sum, AccelZ_g_sum};
  return data; // Return the structure
}


void transmitEarthquake(int8_t n){
  int8_t earthquake[] = {n, };
  HAL_UART_Transmit(&huart2, earthquake, sizeof(earthquake), HAL_MAX_DELAY);
  if (n <= 3) {
    alarm_status1 = n; // Only for debug purposes
  } else {
    alarm_status2 = n; // Only for debug purposes
  }
}
// Test 2: Send "hello world" to master
void test2(UART_HandleTypeDef *uart_handle){
    int8_t hello[12] = "Hello World\n";
    HAL_UART_Transmit(uart_handle, hello, sizeof(hello), HAL_MAX_DELAY);
}

// Test 3: Detect accelerometer data
void test3(uint16_t dev_addr, UART_HandleTypeDef *uart_handle){
    int16_t AccelX, AccelY, AccelZ;
    MPU6050_Read_Accel(dev_addr, &AccelX, &AccelY, &AccelZ);
    int8_t working[1];
    if(AccelX == 0 && AccelY == 0 && AccelZ == 0){
        working[0] = 0;
    } else {
        working[0] = 1;
    }
    HAL_UART_Transmit(uart_handle, working, sizeof(working), HAL_MAX_DELAY);
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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  MPU6050_Init(MPU6050_ADDR1);
  MPU6050_Init(MPU6050_ADDR2);

  // Declare variables to store the calibrated values
  struct CalibrationData calibration1;
  struct CalibrationData calibration2;

  // Call the MPU6050_Calibration function and store the returned structure in the 'calibration' variable
  calibration1 = MPU6050_Calibration(MPU6050_ADDR1);
  calibration2 = MPU6050_Calibration(MPU6050_ADDR2);

  // Access the calibration data
  float AccelX1_cal = calibration1.AccelX;
  float AccelY1_cal = calibration1.AccelY;
  float AccelZ1_cal = calibration1.AccelZ;

  float AccelX2_cal = calibration2.AccelX;
  float AccelY2_cal = calibration2.AccelY;
  float AccelZ2_cal = calibration2.AccelZ;

  float AccelMagSqr1_g;
  float AccelMagSqr2_g;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    MPU6050_Read_Accel(MPU6050_ADDR1, &AccelX1, &AccelY1, &AccelZ1);
    MPU6050_Read_Accel(MPU6050_ADDR2, &AccelX2, &AccelY2, &AccelZ2);

    AccelX1_g = MPU6050_Convert_to_g(AccelX1);
    AccelY1_g = MPU6050_Convert_to_g(AccelY1);
    AccelZ1_g = MPU6050_Convert_to_g(AccelZ1);
    AccelX2_g = MPU6050_Convert_to_g(AccelX2);
    AccelY2_g = MPU6050_Convert_to_g(AccelY2);
    AccelZ2_g = MPU6050_Convert_to_g(AccelZ2);


    AccelX1_g -= AccelX1_cal;
    AccelY1_g -= AccelY1_cal;
    AccelZ1_g -= AccelZ1_cal;
    AccelX2_g -= AccelX2_cal;
    AccelY2_g -= AccelY2_cal;
    AccelZ2_g -= AccelZ2_cal;

    AccelMagSqr1_g = AccelX1_g*AccelX1_g+AccelY1_g*AccelY1_g+AccelZ1_g*AccelZ1_g;
    if(AccelMagSqr1_g>0.2*0.2){
        transmitEarthquake(3);
    } else if(AccelMagSqr1_g>0.1*0.1){
        transmitEarthquake(2);
    } else if(AccelMagSqr1_g>0.02*0.02){
        transmitEarthquake(1);
    } else {
        transmitEarthquake(0);
    }

    AccelMagSqr2_g = AccelX2_g*AccelX2_g+AccelY2_g*AccelY2_g+AccelZ2_g*AccelZ2_g;
    if(AccelMagSqr2_g>0.2*0.2){
        transmitEarthquake(6);
    } else if(AccelMagSqr2_g>0.1*0.1){
        transmitEarthquake(5);
    } else if(AccelMagSqr2_g>0.02*0.02){
        transmitEarthquake(4);
    } else {
        transmitEarthquake(0);
    }


    // n is the severity of earthquake
    // n = 0: no earthquake
    // for mpu1
    // n = 1: >0.02g, <=0.1g
    // n = 2: >0.1g, <=0.2g
    // n = 3: >0.2g
    // for mpu2
    // n = 4: >0.02g, <=0.1g
    // n = 5: >0.1g, <=0.2g
    // n = 6: >0.2g


    // LED BLINKING
    HAL_GPIO_TogglePin (GPIOA, GPIO_PIN_5);
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

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
  huart2.Init.BaudRate = 115200;
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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

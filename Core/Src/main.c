/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention

  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "MPU6050.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
//#include "lcd.h"
int __io_putchar(int ch);
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MPU6050_SAMPLE_INTERVAL_MS 200U
#define UART_COMMAND_BUFFER_SIZE  32U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static uint8_t uartRxByte;
static char uartCommandBuffer[UART_COMMAND_BUFFER_SIZE];
static uint8_t uartCommandLength;
static volatile uint8_t uartCommandReady;
static volatile uint8_t uartCommandOverflow;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void ProcessUartCommand(MPU6050_HandleTypeDef *mpu,
                               uint8_t mpu_address,
                               uint8_t mpu_ready,
                               MPU6050_AccelRange *accel_range,
                               MPU6050_GyroRange *gyro_range);

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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  MPU6050_HandleTypeDef mpu = {0};
  MPU6050_Data sensorData;
  MPU6050_AccelRange accelRange = MPU6050_ACCEL_RANGE_2G;
  MPU6050_GyroRange gyroRange = MPU6050_GYRO_RANGE_250_DPS;
  uint8_t mpuAddress = 0;
  uint8_t mpuReady = 0;
  const uint8_t candidateAddresses[] = {0x68U, 0x69U};
  uint32_t nextSampleTick = HAL_GetTick();
  printf("MPU6050 test start; I2C1 PB6=SCL, PB7=SDA\n");
  if (HAL_UART_Receive_IT(&huart2, &uartRxByte, 1U) != HAL_OK)
  {
    printf("UART RX start error\n");
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if (uartCommandReady)
    {
      ProcessUartCommand(&mpu, mpuAddress, mpuReady, &accelRange, &gyroRange);
    }

    if ((int32_t)(HAL_GetTick() - nextSampleTick) >= 0)
    {
      if (!mpuReady)
      {
        for (uint32_t i = 0; i < sizeof(candidateAddresses) / sizeof(candidateAddresses[0]); i++)
        {
          if (MPU6050_Init(&mpu, &hi2c1, candidateAddresses[i],
                           accelRange,
                           gyroRange,
                           100U) == HAL_OK)
          {
            mpuAddress = candidateAddresses[i];
            mpuReady = 1;
            printf("MPU6050 OK: address=0x%02X WHO_AM_I=0x68\n", mpuAddress);
            printf("RANGE_OK,%u,%u\n", 2U, 250U);
            break;
          }
        }

        if (!mpuReady)
        {
          printf("MPU6050 not found (expected address 0x68/0x69, WHO_AM_I 0x68)\n");
        }
      }
      else if (MPU6050_Read(&mpu, &sensorData, 100U) == HAL_OK)
      {
        printf("ACC=%d,%d,%d TEMP=%d GYRO=%d,%d,%d\n",
               sensorData.accel_x_raw, sensorData.accel_y_raw, sensorData.accel_z_raw,
               sensorData.temperature_raw, sensorData.gyro_x_raw,
               sensorData.gyro_y_raw, sensorData.gyro_z_raw);
        HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
      }
      else
      {
        printf("MPU6050 read error: 0x%08lX\n", HAL_I2C_GetError(&hi2c1));
        mpuReady = 0;
      }

      nextSampleTick = HAL_GetTick() + MPU6050_SAMPLE_INTERVAL_MS;
    }
    else
    {
      __WFI();
    }
  }
  /* USER CODE END 3 */
}

static uint8_t TryGetRanges(int accel_g, int gyro_dps,
                            MPU6050_AccelRange *accel_range,
                            MPU6050_GyroRange *gyro_range)
{
  switch (accel_g)
  {
    case 2:  *accel_range = MPU6050_ACCEL_RANGE_2G; break;
    case 4:  *accel_range = MPU6050_ACCEL_RANGE_4G; break;
    case 8:  *accel_range = MPU6050_ACCEL_RANGE_8G; break;
    case 16: *accel_range = MPU6050_ACCEL_RANGE_16G; break;
    default: return 0U;
  }

  switch (gyro_dps)
  {
    case 250:  *gyro_range = MPU6050_GYRO_RANGE_250_DPS; break;
    case 500:  *gyro_range = MPU6050_GYRO_RANGE_500_DPS; break;
    case 1000: *gyro_range = MPU6050_GYRO_RANGE_1000_DPS; break;
    case 2000: *gyro_range = MPU6050_GYRO_RANGE_2000_DPS; break;
    default: return 0U;
  }

  return 1U;
}

static void ProcessUartCommand(MPU6050_HandleTypeDef *mpu,
                               uint8_t mpu_address,
                               uint8_t mpu_ready,
                               MPU6050_AccelRange *accel_range,
                               MPU6050_GyroRange *gyro_range)
{
  if (uartCommandOverflow)
  {
    printf("RANGE_ERR,COMMAND_TOO_LONG\n");
  }
  else if (strcmp(uartCommandBuffer, "RANGE?") == 0)
  {
    if (mpu_ready)
    {
      const int accel_g = *accel_range == MPU6050_ACCEL_RANGE_2G ? 2 :
                          *accel_range == MPU6050_ACCEL_RANGE_4G ? 4 :
                          *accel_range == MPU6050_ACCEL_RANGE_8G ? 8 : 16;
      const int gyro_dps = *gyro_range == MPU6050_GYRO_RANGE_250_DPS ? 250 :
                           *gyro_range == MPU6050_GYRO_RANGE_500_DPS ? 500 :
                           *gyro_range == MPU6050_GYRO_RANGE_1000_DPS ? 1000 : 2000;
      printf("RANGE_OK,%d,%d\n", accel_g, gyro_dps);
    }
    else
    {
      printf("RANGE_ERR,NOT_READY\n");
    }
  }
  else
  {
    int accel_g = 0;
    int gyro_dps = 0;
    char trailing_character = '\0';
    MPU6050_AccelRange new_accel_range;
    MPU6050_GyroRange new_gyro_range;
    const int parsed_fields = sscanf(uartCommandBuffer, "RANGE,%d,%d%c",
                                     &accel_g, &gyro_dps, &trailing_character);

    if (parsed_fields != 2 ||
        !TryGetRanges(accel_g, gyro_dps, &new_accel_range, &new_gyro_range))
    {
      printf("RANGE_ERR,BAD_COMMAND\n");
    }
    else if (!mpu_ready)
    {
      printf("RANGE_ERR,NOT_READY\n");
    }
    else if (MPU6050_Init(mpu, &hi2c1, mpu_address, new_accel_range,
                          new_gyro_range, 100U) == HAL_OK)
    {
      *accel_range = new_accel_range;
      *gyro_range = new_gyro_range;
      printf("RANGE_OK,%d,%d\n", accel_g, gyro_dps);
    }
    else
    {
      printf("RANGE_ERR,CONFIG\n");
    }
  }

  __disable_irq();
  uartCommandReady = 0U;
  uartCommandOverflow = 0U;
  uartCommandLength = 0U;
  __enable_irq();
  (void)HAL_UART_Receive_IT(&huart2, &uartRxByte, 1U);
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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

/* USER CODE BEGIN 4 */
int __io_putchar(int ch)
{
  if (ch == '\n')
  {
    uint8_t carriageReturn = '\r';
    HAL_UART_Transmit(&huart2, &carriageReturn, 1, HAL_MAX_DELAY);
  }

  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance != USART2)
  {
    return;
  }

  if (uartRxByte == '\n')
  {
    uartCommandBuffer[uartCommandLength] = '\0';
    uartCommandReady = 1U;
  }
  else if (uartRxByte != '\r' && !uartCommandOverflow)
  {
    if (uartCommandLength < sizeof(uartCommandBuffer) - 1U)
    {
      uartCommandBuffer[uartCommandLength++] = (char)uartRxByte;
    }
    else
    {
      uartCommandOverflow = 1U;
    }
  }

  if (!uartCommandReady)
  {
    (void)HAL_UART_Receive_IT(&huart2, &uartRxByte, 1U);
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

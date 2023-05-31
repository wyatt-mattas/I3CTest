/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
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
#include "bmi3.h"
#include "bmi323.h"
#include "common.h"
#include <stdio.h>
#include <math.h>
#include "i3c_handle.h"
#include "desc_target1.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/*! Earth's gravity in m/s^2 */
#define GRAVITY_EARTH (9.80665f)

#define ACCEL UINT8_C(0x00)
#define GYRO UINT8_C(0x01)
#define TEMPERATURE UINT8_C(0x02)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I3C_HandleTypeDef hi3c1;

__IO uint32_t uwTargetCount = 0;

/* USER CODE BEGIN PV */
/******************************************************************************/
/*!         Structure Definition                                              */

/*! Structure to define accelerometer and gyroscope configuration. */
struct bmi3_sens_config config[2];

/******************************************************************************/
/*!         Static Function Declaration                                       */

/*!
 *  @brief This internal API is used to set configurations for FIFO, accelerometer and gyroscope.
 *
 *  @param[in] dev       : Structure instance of bmi3_dev.
 *
 *  @return Status of execution.
 */
static int8_t set_sensor_config(struct bmi3_dev *dev);

/*!
 *  @brief This internal function converts lsb to meter per second squared for 16 bit accelerometer for
 *  range 2G, 4G, 8G or 16G.
 *
 *  @param[in] val       : LSB from each axis.
 *  @param[in] g_range   : Gravity range.
 *  @param[in] bit_width : Resolution for accel.
 *
 *  @return Accel values in meter per second squared.
 */
static float lsb_to_mps2(int16_t val, int8_t g_range, uint8_t bit_width);

/*!
 *  @brief This function converts lsb to degree per second for 16 bit gyro at
 *  range 125, 250, 500, 1000 or 2000dps.
 *
 *  @param[in] val       : LSB from each axis.
 *  @param[in] dps       : Degree per second.
 *  @param[in] bit_width : Resolution for gyro.
 *
 *  @return Degree per second.
 */
static float lsb_to_dps(int16_t val, float dps, uint8_t bit_width);
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I3C1_Init(void);
static void MX_ICACHE_Init(void);
/* USER CODE BEGIN PFP */
/* Array contain targets descriptor */
TargetDesc_TypeDef *aTargetDesc[1] =
    {
        &TargetDesc1, /* DEVICE_ID1 */
};

/* Buffer that contain payload data, mean PID, BCR, DCR */
uint8_t aPayloadBuffer[64 * COUNTOF(aTargetDesc)];

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
  MX_I3C1_Init();
  MX_ICACHE_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  struct bmi3_dev dev;
  int8_t rslt;

  /* Variable to set the data sample rate of i3c sync
   * 0x0032 is set to 50 samples this value can vary  */
  uint16_t sample_rate = 0x0032;

  /* Variable to set the delay time of i3c sync */
  uint8_t delay_time = BMI3_I3C_SYNC_DIVISION_FACTOR_11;

  /* Variable to set the i3c sync ODR */
  uint8_t odr = BMI3_I3C_SYNC_ODR_50HZ;

  /* Variable to enable the filer */
  uint8_t i3c_tc_res = BMI323_ENABLE;

  uint8_t limit = 20;

  uint16_t int_status;

  struct bmi3_feature_enable feature = {0};

  struct bmi3_sensor_data sensor_data[3];

  /* Variable to store temperature */
  float temperature_value;

  float acc_x = 0, acc_y = 0, acc_z = 0;
  float gyr_x = 0, gyr_y = 0, gyr_z = 0;
  uint8_t indx = 0;

  /* Function to select interface between SPI and I2C, according to that the device structure gets updated.
   * Interface reference is given as a parameter
   * For I2C : BMI3_I2C_INTF
   * For SPI : BMI3_SPI_INTF
   */
  rslt = bmi3_interface_init(&dev, BMI3_I3C_INTF);
  bmi3_error_codes_print_result("bmi3_interface_init", rslt);

  if (rslt == BMI323_OK)
  {
    /* Initialize BMI323 */
    rslt = bmi323_init(&dev);
    bmi3_error_codes_print_result("bmi323_init", rslt);

    if (rslt == BMI323_OK)
    {
      rslt = bmi323_configure_enhanced_flexibility(&dev);
      bmi3_error_codes_print_result("bmi323_configure_enhanced_flexibility", rslt);

      if (rslt == BMI323_OK)
      {
        rslt = set_sensor_config(&dev);
        bmi3_error_codes_print_result("set_sensor_config", rslt);

        /* Enable i3c_sync feature */
        feature.i3c_sync_en = BMI323_ENABLE;

        /* Enable the selected sensors */
        rslt = bmi323_select_sensor(&feature, &dev);
        bmi3_error_codes_print_result("bmi323_select_sensor", rslt);

        /* Set the data sample rate of i3c sync */
        rslt = bmi323_set_i3c_tc_sync_tph(sample_rate, &dev);
        bmi3_error_codes_print_result("set_i3c_tc_sync_tph", rslt);

        /* Set the delay time of i3c sync */
        rslt = bmi323_set_i3c_tc_sync_tu(delay_time, &dev);
        bmi3_error_codes_print_result("set_i3c_tc_sync_tu", rslt);

        /* Set i3c sync ODR */
        rslt = bmi323_set_i3c_tc_sync_odr(odr, &dev);
        bmi3_error_codes_print_result("set_i3c_tc_sync_odr", rslt);

        /* Enable the i3c sync filter */
        rslt = bmi323_set_i3c_sync_i3c_tc_res(i3c_tc_res, &dev);
        bmi3_error_codes_print_result("set_i3c_sync_i3c_tc_res", rslt);

        /* After any change to the I3C-Sync configuration parameters,
         * a config changed CMD (0x0201) must be written
         * to CMD register to update the internal configuration */
        rslt = bmi323_set_command_register(BMI3_CMD_I3C_TCSYNC_UPDATE, &dev);
        bmi3_error_codes_print_result("bmi323_set_command_register", rslt);

        /* Set the type of sensor data */
        sensor_data[ACCEL].type = BMI3_I3C_SYNC_ACCEL;
        sensor_data[GYRO].type = BMI3_I3C_SYNC_GYRO;
        sensor_data[TEMPERATURE].type = BMI3_I3C_SYNC_TEMP;

        printf("I3C accel, gyro and temperature data\n");

        printf(
            "\nDATA_SET, i3c_acc_x, i3c_acc_y, i3c_acc_z, i3c_time, Gravity-x, Gravity-y, Gravity-z, i3c_gyro_x, i3c_gyro_y, i3c_gyro_z, DPS-x, DPS-y, DPS-z, i3c_temperature, i3c_sync_time, Temperature data(Degree Celcius)\n");

        while (indx <= limit)
        {
          /* Delay provided based on i3c sync ODR */
          dev.delay_us(20000, dev.intf_ptr);

          /* To get the status of i3c sync interrupt status. */
          rslt = bmi323_get_int1_status(&int_status, &dev);
          bmi3_error_codes_print_result("bmi323_get_int1_status", rslt);

          /* To check the i3c sync accel data */
          if (int_status & BMI3_INT_STATUS_I3C)
          {
            rslt = bmi323_get_sensor_data(sensor_data, 3, &dev);
            bmi3_error_codes_print_result("bmi323_get_sensor_data", rslt);

            /* Converting lsb to meter per second squared for 16 bit accelerometer at 2G range. */
            acc_x = lsb_to_mps2((int16_t)sensor_data[ACCEL].sens_data.i3c_sync.sync_x, 2, dev.resolution);
            acc_y = lsb_to_mps2((int16_t)sensor_data[ACCEL].sens_data.i3c_sync.sync_y, 2, dev.resolution);
            acc_z = lsb_to_mps2((int16_t)sensor_data[ACCEL].sens_data.i3c_sync.sync_z, 2, dev.resolution);

            /* Converting lsb to degree per second for 16 bit gyro at 2000dps range. */
            gyr_x = lsb_to_dps((int16_t)sensor_data[GYRO].sens_data.i3c_sync.sync_x,
                               (float)2000,
                               dev.resolution);
            gyr_y = lsb_to_dps((int16_t)sensor_data[GYRO].sens_data.i3c_sync.sync_y,
                               (float)2000,
                               dev.resolution);
            gyr_z = lsb_to_dps((int16_t)sensor_data[GYRO].sens_data.i3c_sync.sync_z,
                               (float)2000,
                               dev.resolution);

            temperature_value =
                (float)((((float)((int16_t)sensor_data[TEMPERATURE].sens_data.i3c_sync.sync_temp)) /
                         512.0) +
                        23.0);

            printf(
                "%d, %d, %d, %d, %d, %4.2f, %4.2f, %4.2f, %d, %d, %d, %d, %4.2f, %4.2f, %4.2f, %d, %d, %f\t\n",
                indx,
                sensor_data[ACCEL].sens_data.i3c_sync.sync_x,
                sensor_data[ACCEL].sens_data.i3c_sync.sync_y,
                sensor_data[ACCEL].sens_data.i3c_sync.sync_z,
                sensor_data[ACCEL].sens_data.i3c_sync.sync_time,
                acc_x,
                acc_y,
                acc_z,
                sensor_data[GYRO].sens_data.i3c_sync.sync_x,
                sensor_data[GYRO].sens_data.i3c_sync.sync_y,
                sensor_data[GYRO].sens_data.i3c_sync.sync_z,
                sensor_data[GYRO].sens_data.i3c_sync.sync_time,
                gyr_x,
                gyr_y,
                gyr_z,
                sensor_data[TEMPERATURE].sens_data.i3c_sync.sync_temp,
                sensor_data[TEMPERATURE].sens_data.i3c_sync.sync_time,
                temperature_value);
          }

          indx++;
        }
      }
    }
  }

  // bmi3_coines_deinit();

  return rslt;
}

/*!
 * @brief This internal API is used to set configurations for accelerometer, gyroscope and FIFO.
 */
static int8_t set_sensor_config(struct bmi3_dev *dev)
{
  int8_t rslt;

  /* Structure to define interrupt with its feature */
  struct bmi3_map_int map_int = {0};

  /* Configure type of feature */
  config[ACCEL].type = BMI323_ACCEL;
  config[GYRO].type = BMI323_GYRO;

  /* Get default configurations for the type of feature selected */
  rslt = bmi323_get_sensor_config(config, 2, dev);
  bmi3_error_codes_print_result("bmi323_get_sensor_config", rslt);

  /* Configure the accel and gyro settings */
  config[ACCEL].cfg.acc.bwp = BMI3_ACC_BW_ODR_QUARTER;
  config[ACCEL].cfg.acc.range = BMI3_ACC_RANGE_2G;
  config[ACCEL].cfg.acc.acc_mode = BMI3_ACC_MODE_NORMAL;

  config[GYRO].cfg.gyr.bwp = BMI3_GYR_BW_ODR_HALF;
  config[GYRO].cfg.gyr.range = BMI3_GYR_RANGE_125DPS;
  config[GYRO].cfg.gyr.gyr_mode = BMI3_GYR_MODE_NORMAL;

  rslt = bmi323_set_sensor_config(config, 2, dev);
  bmi3_error_codes_print_result("bmi323_set_sensor_config", rslt);

  /* Get configurations for the type of feature selected */
  rslt = bmi323_get_sensor_config(config, 2, dev);
  bmi3_error_codes_print_result("bmi323_get_sensor_config", rslt);

  /* Select the feature and map the interrupt to pin BMI3_INT1 or BMI3_INT2 */
  map_int.i3c_out = BMI3_INT1;

  /* Map i3c sync interrupt to interrupt pin. */
  rslt = bmi323_map_interrupt(map_int, dev);
  bmi3_error_codes_print_result("bmi323_map_interrupt", rslt);

  return rslt;
}

/*!
 * @brief This function converts lsb to meter per second squared for 16 bit accelerometer at
 * range 2G, 4G, 8G or 16G.
 */
static float lsb_to_mps2(int16_t val, int8_t g_range, uint8_t bit_width)
{
  double power = 2;

  float half_scale = (float)((pow((double)power, (double)bit_width) / 2.0f));

  return (GRAVITY_EARTH * val * g_range) / half_scale;
}

/*!
 * @brief This function converts lsb to degree per second for 16 bit gyro at
 * range 125, 250, 500, 1000 or 2000dps.
 */
static float lsb_to_dps(int16_t val, float dps, uint8_t bit_width)
{
  double power = 2;

  float half_scale = (float)((pow((double)power, (double)bit_width) / 2.0f));

  return (dps / (half_scale)) * (val);
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY))
  {
  }

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS_DIGITAL;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLL1_SOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 250;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1_VCIRANGE_1;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1_VCORANGE_WIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 | RCC_CLOCKTYPE_PCLK3;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
 * @brief I3C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I3C1_Init(void)
{

  /* USER CODE BEGIN I3C1_Init 0 */

  /* USER CODE END I3C1_Init 0 */

  I3C_FifoConfTypeDef sFifoConfig = {0};
  I3C_CtrlConfTypeDef sCtrlConfig = {0};

  /* USER CODE BEGIN I3C1_Init 1 */

  /* USER CODE END I3C1_Init 1 */
  hi3c1.Instance = I3C1;
  hi3c1.Mode = HAL_I3C_MODE_CONTROLLER;
  hi3c1.Init.CtrlBusCharacteristic.SDAHoldTime = HAL_I3C_SDA_HOLD_TIME_1_5;
  hi3c1.Init.CtrlBusCharacteristic.WaitTime = HAL_I3C_OWN_ACTIVITY_STATE_0;
  hi3c1.Init.CtrlBusCharacteristic.SCLPPLowDuration = 0x09;
  hi3c1.Init.CtrlBusCharacteristic.SCLI3CHighDuration = 0x09;
  hi3c1.Init.CtrlBusCharacteristic.SCLODLowDuration = 0xae;
  hi3c1.Init.CtrlBusCharacteristic.SCLI2CHighDuration = 0x4a;
  hi3c1.Init.CtrlBusCharacteristic.BusFreeDuration = 0x83;
  hi3c1.Init.CtrlBusCharacteristic.BusIdleDuration = 0xf8;
  if (HAL_I3C_Init(&hi3c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure FIFO
   */
  sFifoConfig.RxFifoThreshold = HAL_I3C_RXFIFO_THRESHOLD_1_4;
  sFifoConfig.TxFifoThreshold = HAL_I3C_TXFIFO_THRESHOLD_1_4;
  sFifoConfig.ControlFifo = HAL_I3C_CONTROLFIFO_DISABLE;
  sFifoConfig.StatusFifo = HAL_I3C_STATUSFIFO_DISABLE;
  if (HAL_I3C_SetConfigFifo(&hi3c1, &sFifoConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure controller
   */
  sCtrlConfig.DynamicAddr = 0;
  sCtrlConfig.StallTime = 0x00;
  sCtrlConfig.HotJoinAllowed = DISABLE;
  sCtrlConfig.ACKStallState = DISABLE;
  sCtrlConfig.CCCStallState = DISABLE;
  sCtrlConfig.TxStallState = DISABLE;
  sCtrlConfig.RxStallState = DISABLE;
  sCtrlConfig.HighKeeperSDA = DISABLE;
  if (HAL_I3C_Ctrl_Config(&hi3c1, &sCtrlConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I3C1_Init 2 */

  /* USER CODE END I3C1_Init 2 */
}

/**
 * @brief ICACHE Initialization Function
 * @param None
 * @retval None
 */
static void MX_ICACHE_Init(void)
{

  /* USER CODE BEGIN ICACHE_Init 0 */

  /* USER CODE END ICACHE_Init 0 */

  /* USER CODE BEGIN ICACHE_Init 1 */

  /* USER CODE END ICACHE_Init 1 */

  /** Enable instruction cache in 1-way (direct mapped cache)
   */
  if (HAL_ICACHE_ConfigAssociativityMode(ICACHE_1WAY) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_ICACHE_Enable() != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ICACHE_Init 2 */

  /* USER CODE END ICACHE_Init 2 */
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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED2_YELLOW_GPIO_Port, LED2_YELLOW_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED1_GREEN_GPIO_Port, LED1_GREEN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED3_RED_GPIO_Port, LED3_RED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_BUTTON_Pin */
  GPIO_InitStruct.Pin = USER_BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_BUTTON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED2_YELLOW_Pin */
  GPIO_InitStruct.Pin = LED2_YELLOW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED2_YELLOW_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_MDC_Pin RMII_RXD0_Pin RMII_RXD1_Pin */
  GPIO_InitStruct.Pin = RMII_MDC_Pin | RMII_RXD0_Pin | RMII_RXD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_REF_CLK_Pin RMII_MDIO_Pin RMII_CRS_DV_Pin */
  GPIO_InitStruct.Pin = RMII_REF_CLK_Pin | RMII_MDIO_Pin | RMII_CRS_DV_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : VBUS_SENSE_Pin */
  GPIO_InitStruct.Pin = VBUS_SENSE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(VBUS_SENSE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED1_GREEN_Pin */
  GPIO_InitStruct.Pin = LED1_GREEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED1_GREEN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : UCPD_CC1_Pin UCPD_CC2_Pin */
  GPIO_InitStruct.Pin = UCPD_CC1_Pin | UCPD_CC2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : RMII_TXD1_Pin */
  GPIO_InitStruct.Pin = RMII_TXD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(RMII_TXD1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : T_VCP_TX_Pin T_VCP_RX_Pin */
  GPIO_InitStruct.Pin = T_VCP_TX_Pin | T_VCP_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : LED3_RED_Pin */
  GPIO_InitStruct.Pin = LED3_RED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED3_RED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : UCPD_FLT_Pin */
  GPIO_InitStruct.Pin = UCPD_FLT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(UCPD_FLT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_FS_N_Pin USB_FS_P_Pin */
  GPIO_InitStruct.Pin = USB_FS_N_Pin | USB_FS_P_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_USB;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_TXT_EN_Pin RMI_TXD0_Pin */
  GPIO_InitStruct.Pin = RMII_TXT_EN_Pin | RMI_TXD0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D1_TX_Pin ARD_D0_RX_Pin */
  GPIO_InitStruct.Pin = ARD_D1_TX_Pin | ARD_D0_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF8_LPUART1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/**
 * @brief I3C target request a dynamic address callback.
 *        The main objective of this user function is to check if a target request a dynamic address.
 *        if the case we should assign a dynamic address to the target.
 * @par Called functions
 * - HAL_I3C_TgtReqDynamicAddrCallback()
 * - HAL_I3C_Ctrl_SetDynamicAddress()
 * @retval None
 */
void HAL_I3C_TgtReqDynamicAddrCallback(I3C_HandleTypeDef *hi3c, uint64_t targetPayload)
{
  /* Update Payload on aTargetDesc */
  aTargetDesc[uwTargetCount]->TARGET_BCR_DCR_PID = targetPayload;

  /* Send associated dynamic address */
  HAL_I3C_Ctrl_SetDynAddr(hi3c, aTargetDesc[uwTargetCount++]->DYNAMIC_ADDR);
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

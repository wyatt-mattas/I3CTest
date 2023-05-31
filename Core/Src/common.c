/**
 * Copyright (C) 2023 Bosch Sensortec GmbH. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "bmi3.h"
#include "main.h"
#include "common.h"
#include "dwt_delay.h"
// #include "i3c_handle.h"

// #include "stm32f0xx_ll_i2c.h"

/******************************************************************************/
/*!                Macro definition                                           */

#define READ_WRITE_LEN UINT8_C(8)

/******************************************************************************/
/*!                Static variable definition                                 */

/*! Variable that holds the I2C device address or SPI chip selection */
static uint8_t dev_addr = 0x32;

extern I3C_HandleTypeDef hi3c1;

volatile uint32_t time_us = 0;

/******************************************************************************/
/*!                Static function definition                                 */

/*!
 * @brief This internal API reads I2C function map to COINES platform
 *
 * @param[in]     reg_addr : 8bit register address of the sensor
 * @param[out]    reg_data : Data from the specified address
 * @param[in]     len      : Length of the reg_data array
 * @param[in,out] intf_ptr : Void pointer that can enable the linking of descriptors
 *                           for interface related callbacks.
 *
 * @return Status of execution.
 */
static BMI3_INTF_RET_TYPE I3C_Read_IT(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr);

/*!
 * @brief This internal API writes I2C function map to COINES platform
 *
 * @param[in]     reg_addr : 8bit register address of the sensor
 * @param[out]    reg_data : Data from the specified address
 * @param[in]     len      : Length of the reg_data array
 * @param[in,out] intf_ptr : Void pointer that can enable the linking of descriptors
 *                           for interface related callbacks.
 *
 * @return Status of execution.
 */
static BMI3_INTF_RET_TYPE I3C_Write_IT(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr);

/*!
 * @brief This internal API maps delay function to COINES platform
 *
 * @param[in] period       : The time period in microseconds
 * @param[in,out] intf_ptr : Void pointer that can enable the linking of descriptors
 *                           for interface related callbacks
 *
 * @return void.
 */
static void bmi3_delay_us(uint32_t period, void *intf_ptr);

// void i2c_scan(I2C_HandleTypeDef *);

int8_t configure_i3c_bus_devices(int8_t device_index, uint8_t dev_addr);

/* Number of Targets detected during DAA procedure */
// __IO uint32_t uwTargetCount = 0;

// /* Variable to catch HotJoin event */
// __IO uint32_t uwHotJoinRequested = 0;

// /* Buffer that contain payload data, mean PID, BCR, DCR */
// uint8_t aPayloadBuffer[64 * COUNTOF(aTargetDesc)];

/******************************************************************************/
/*!               User interface functions                                    */

/*!
 * @brief This API prints the execution status
 */
void bmi3_error_codes_print_result(const char api_name[], int8_t rslt)
{
    switch (rslt)
    {
    case BMI3_OK:

        /*! Do nothing */
        break;

    case BMI3_E_NULL_PTR:
        printf("%s\t", api_name);
        printf(
            "Error [%d] : Null pointer error. It occurs when the user tries to assign value (not address) to a pointer,"
            " which has been initialized to NULL.\r\n",
            rslt);
        break;

    case BMI3_E_COM_FAIL:
        printf("%s\t", api_name);
        printf(
            "Error [%d] : Communication failure error. It occurs due to read/write operation failure and also due "
            "to power failure during communication\r\n",
            rslt);
        break;

    case BMI3_E_DEV_NOT_FOUND:
        printf("%s\t", api_name);
        printf("Error [%d] : Device not found error. It occurs when the device chip id is incorrectly read\r\n",
               rslt);
        break;

    case BMI3_E_INVALID_SENSOR:
        printf("%s\t", api_name);
        printf(
            "Error [%d] : Invalid sensor error. It occurs when there is a mismatch in the requested feature with the "
            "available one\r\n",
            rslt);
        break;

    case BMI3_E_INVALID_INT_PIN:
        printf("%s\t", api_name);
        printf(
            "Error [%d] : Invalid interrupt pin error. It occurs when the user tries to configure interrupt pins "
            "apart from INT1 and INT2\r\n",
            rslt);
        break;

    case BMI3_E_ACC_INVALID_CFG:
        printf("%s\t", api_name);
        printf(
            "Error [%d] : Invalid Accel configuration error. It occurs when there is an error in accel configuration"
            " register which could be one among range, BW or filter performance in reg address 0x20\r\n",
            rslt);
        break;

    case BMI3_E_GYRO_INVALID_CFG:
        printf("%s\t", api_name);
        printf(
            "Error [%d] : Invalid Gyro configuration error. It occurs when there is a error in gyro configuration"
            "register which could be one among range, BW or filter performance in reg address 0x21\r\n",
            rslt);
        break;

    case BMI3_E_INVALID_INPUT:
        printf("%s\t", api_name);
        printf("Error [%d] : Invalid input error. It occurs when the sensor input validity fails\r\n", rslt);
        break;

    case BMI3_E_INVALID_STATUS:
        printf("%s\t", api_name);
        printf("Error [%d] : Invalid status error. It occurs when the feature/sensor validity fails\r\n", rslt);
        break;

    case BMI3_E_DATA_RDY_INT_FAILED:
        printf("%s\t", api_name);
        printf(
            "Error [%d] : Data ready interrupt error. It occurs when the sample count exceeds the FOC sample limit "
            "and data ready status is not updated\r\n",
            rslt);
        break;

    case BMI3_E_INVALID_FOC_POSITION:
        printf("%s\t", api_name);
        printf(
            "Error [%d] : Invalid FOC position error. It occurs when average FOC data is obtained for the wrong"
            " axes\r\n",
            rslt);
        break;

    case BMI3_E_INVALID_ST_SELECTION:
        printf("%s\t", api_name);
        printf(
            "Error [%d] : Invalid self-test selection error. It occurs when there is an invalid precondition"
            "settings such as alternate accelerometer and gyroscope enable bits, accelerometer mode and output data rate\r\n",
            rslt);
        break;

    case BMI3_E_OUT_OF_RANGE:
        printf("%s\t", api_name);
        printf(
            "Error [%d] : Out of range error. It occurs when the range exceeds the maximum range for accel while performing FOC\r\n",
            rslt);
        break;

    case BMI3_E_FEATURE_ENGINE_STATUS:
        printf("%s\t", api_name);
        printf(
            "Error [%d] : Feature engine status error. It occurs when the feature engine enable mask is not set\r\n",
            rslt);
        break;

    default:
        printf("%s\t", api_name);
        printf("Error [%d] : Unknown error code\r\n", rslt);
        break;
    }
}

/*!
 * @brief This function is to select the interface between SPI and I2C.
 */
int8_t bmi3_interface_init(struct bmi3_dev *dev, int8_t intf)
{
    // i2c_scan(&hi2c1);

    int8_t rslt = BMI3_OK;

    if (dev != NULL)
    {
        if (intf == BMI3_I3C_INTF)
        {
            dev->read = I3C_Read_IT;
            dev->write = I3C_Write_IT;
            dev->intf = BMI3_I3C_INTF;

            rslt = HAL_I3C_Ctrl_DynAddrAssign_IT(&hi3c1, I3C_ONLY_ENTDAA);

            if (rslt != HAL_OK)
            {
                /* Error_Handler() function is called when error occurs. */
                Error_Handler();
            }
            printf("I3C bus exited LISTEN state\n");
        }

        // /* Bus configuration : I2C */
        // if (intf == BMI3_I2C_INTF)
        // {
        //     dev_addr = BMI3_ADDR_I2C_PRIM;
        //     dev->read = bmi3_i2c_read;
        //     dev->write = bmi3_i2c_write;
        //     dev->intf = BMI3_I2C_INTF;
        // }

        dwt_delay_init();

        /* Configure delay in microseconds */
        dev->delay_us = dwt_delay_us;

        /* Assign device address to interface pointer */
        dev->intf_ptr = &dev_addr;

        /* Configure max read/write length (in bytes) ( Supported length depends on target machine) */
        dev->read_write_len = READ_WRITE_LEN;
    }
    else
    {
        rslt = BMI3_E_NULL_PTR;
    }

    return rslt;
}

int8_t configure_i3c_bus_devices(int8_t device_index, uint8_t dev_addr)
{

    int8_t rslt = BMI3_OK;

    I3C_DeviceConfTypeDef device_conf[1]; // Replace 1 with the number of devices you want to configure

    // Configure the first device
    device_conf[0].DeviceIndex = device_index;   // Device index in the DEVRx register, should be between 1 and 4
    device_conf[0].TargetDynamicAddr = dev_addr; // Use the dynamic address assigned to the target device (dev_addr)
    device_conf[0].IBIAck = ENABLE;              // Enable/Disable controller's ACK when receiving an IBI
    device_conf[0].IBIPayload = DISABLE;         // Enable/Disable controller's receiving IBI payload after acknowledging an IBI request
    device_conf[0].CtrlRoleReqAck = ENABLE;      // Enable/Disable controller's ACK when receiving a control request
    device_conf[0].CtrlStopTransfer = DISABLE;   // Enable/Disable controller's stop transfer after receiving an IBI request

    // Configure the other devices if needed

    // Call HAL_I3C_Ctrl_ConfigBusDevices() to store the target capabilities in the hardware register
    int8_t status = HAL_I3C_Ctrl_ConfigBusDevices(&hi3c1, device_conf, 1); // Replace 1 with the number of devices you're configuring

    if (status != HAL_OK)
    {
        rslt = status;
    }

    return rslt;
}

// void i2c_scan(I2C_HandleTypeDef *hi2c1)
// {
//     uint8_t dummy_data = 0;
//     uint8_t device_addr;
//     int8_t status;

//     // printf("Scanning I2C bus:\n");
//     for (device_addr = 0; device_addr < 127; device_addr++)
//     {
//         status = HAL_I2C_Master_Transmit(hi2c1, (uint8_t)(device_addr << 1), &dummy_data, 1, 1000);
//         if (status == HAL_OK)
//         {
//             printf("Found device at address: 0x%02X)\n", device_addr);
//         }
//     }
// }

/******************************************************************************/
/*!               Static functions                                            */

/*!
 * @brief This internal API reads I2C function map to COINES platform
 */

static BMI3_INTF_RET_TYPE I3C_Read_IT(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    // Assuming intf_ptr is used to point to the I3C_HandleTypeDef
    I3C_HandleTypeDef *hi3c = (I3C_HandleTypeDef *)intf_ptr;

    I3C_XferTypeDef pXferData;
    pXferData.RxBuf.pBuffer = reg_data;
    pXferData.RxBuf.Size = len;

    pXferData.CtrlBuf.pBuffer = &reg_addr;
    pXferData.CtrlBuf.Size = 1;

    return HAL_I3C_Ctrl_Receive_IT(hi3c, &pXferData);
}

static BMI3_INTF_RET_TYPE I3C_Write_IT(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    // Assuming intf_ptr is used to point to the I3C_HandleTypeDef
    I3C_HandleTypeDef *hi3c = (I3C_HandleTypeDef *)intf_ptr;

    I3C_XferTypeDef pXferData;
    pXferData.TxBuf.pBuffer = reg_data;
    pXferData.TxBuf.Size = len;

    pXferData.CtrlBuf.pBuffer = &reg_addr;
    pXferData.CtrlBuf.Size = 1;

    return HAL_I3C_Ctrl_Transmit_IT(hi3c, &pXferData);
}

// static BMI3_INTF_RET_TYPE bmi3_i3c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
// {
//     uint8_t device_addr = *(uint8_t *)intf_ptr;

//     (void)intf_ptr;

//     int8_t rslt = 0; /* Return 0 for Success, non-zero for failure */

//     HAL_StatusTypeDef status = HAL_OK;

//     // Prepare I3C_XferTypeDef structure
//     I3C_XferTypeDef xferData;

//     // Control buffer (reg_addr)
//     uint32_t ctrlBuf = (uint32_t)reg_addr;
//     xferData.CtrlBuf.pBuffer = &ctrlBuf;
//     xferData.CtrlBuf.Size = 1;

//     // Data buffer (reg_data)
//     xferData.TxBuf.pBuffer = NULL; // Not needed for read operation
//     xferData.TxBuf.Size = 0;

//     // Status buffer (not needed for read operation)
//     xferData.StatusBuf.pBuffer = NULL;
//     xferData.StatusBuf.Size = 0;

//     // Receive buffer (reg_data)
//     xferData.RxBuf.pBuffer = reg_data;
//     xferData.RxBuf.Size = len;

//     // Transmit the data
//     status = HAL_I3C_Ctrl_Transmit(&hi3c1, &xferData, 1000);
//     if (status != HAL_OK)
//     {
//         rslt = (-1);
//     }

//     // Receive the data
//     status = HAL_I3C_Ctrl_Receive(&hi3c1, &xferData, 10000);
//     if (status != HAL_OK)
//     {
//         rslt = (-1);
//     }

//     return rslt;
// }

// /*!
//  * @brief This internal API writes I2C function map to COINES platform
//  */
// static BMI3_INTF_RET_TYPE bmi3_i3c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
// {
//     uint8_t device_addr = *(uint8_t *)intf_ptr;

//     (void)intf_ptr;

//     int8_t rslt = 0; /* Return 0 for Success, non-zero for failure */

//     HAL_StatusTypeDef status = HAL_OK;

//     // Prepare I3C_XferTypeDef structure
//     I3C_XferTypeDef xferData;

//     // Control buffer (reg_addr)
//     uint32_t ctrlBuf = (uint32_t)reg_addr;
//     xferData.CtrlBuf.pBuffer = &ctrlBuf;
//     xferData.CtrlBuf.Size = 1;

//     // Data buffer (reg_data)
//     xferData.TxBuf.pBuffer = (uint8_t *)reg_data;
//     xferData.TxBuf.Size = len;

//     // Status buffer (not needed for transmit)
//     xferData.StatusBuf.pBuffer = NULL;
//     xferData.StatusBuf.Size = 0;

//     // Receive buffer (not needed for transmit)
//     xferData.RxBuf.pBuffer = NULL;
//     xferData.RxBuf.Size = 0;

//     // Transmit the data
//     status = HAL_I3C_Ctrl_Transmit(&hi3c1, &xferData, 1000);

//     if (status != HAL_OK)
//     {
//         rslt = (-1);
//     }

//     return rslt;
// }

// static void bmi3_delay_us(uint32_t us, void *intf_ptr)
// {
//     (void)intf_ptr;

//     // Assuming your system core clock is set correctly
//     uint32_t sysClock = SystemCoreClock;

//     // Calculate the number of loops required for the specified delay
//     // Each loop takes 5 cycles: 1 for SUB, 1 for CMP, 1 for BNE, and 2 for the branch
//     uint32_t loopCount = (us * sysClock) / 5000000;

//     // Create the delay loop
//     asm volatile(
//         "1: \n\t"
//         "   SUB %[loopCount], #1 \n\t"
//         "   CMP %[loopCount], #0 \n\t"
//         "   BNE 1b \n\t"
//         : [loopCount] "+l"(loopCount));
// }

// void SysTick_Init(uint32_t ticks)
// {
//     SysTick->LOAD = (uint32_t)(ticks - 1UL);                                                         // Set the reload value
//     SysTick->VAL = 0UL;                                                                              // Reset the SysTick counter value
//     SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk; // Enable SysTick, use processor clock, enable SysTick interrupt
// }

// void SysTick_Handler(void)
// {
//     time_us++;
// }

// void delay_us(uint32_t us, void *intf_ptr)
// {
//     (void)intf_ptr;

//     uint32_t start_time = time_us;
//     while ((time_us - start_time) < us)
//         ;
// }
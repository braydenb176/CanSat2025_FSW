#include "ICM42688PSPI.h"
#include "stm32g4xx_hal.h"
#include <stdint.h>

static SPI_HandleTypeDef* hspi;

/* Private GPIO CS Pin Variables */
static GPIO_TypeDef* ChipSelect_GPIO_Port;
static uint16_t ChipSelect_Pin;

static void ICM42688P_disable_chip_select()
{
    HAL_GPIO_WritePin(ChipSelect_GPIO_Port, ChipSelect_Pin, GPIO_PIN_RESET);
}

static void ICM42688P_enable_chip_select()
{
    HAL_GPIO_WritePin(ChipSelect_GPIO_Port, ChipSelect_Pin, GPIO_PIN_SET);
}

static HAL_StatusTypeDef ICM42688P_write_reg(uint8_t reg, uint8_t data)
{
    uint8_t tx[2] = {reg, data};
    ICM42688P_disable_chip_select();
    HAL_SPI_Transmit(hspi, tx, 2, HAL_MAX_DELAY);
    ICM42688P_enable_chip_select();
    return HAL_OK;
}

uint8_t ICM42688P_read_reg(uint8_t reg)
{
    uint8_t tx = reg | (1 << 7);
    uint8_t rx = 0;
    ICM42688P_disable_chip_select();
    HAL_SPI_Transmit(hspi, &tx, 1, HAL_MAX_DELAY);
    
    HAL_SPI_Receive(hspi, &rx, 1, HAL_MAX_DELAY);
    ICM42688P_enable_chip_select();
    return rx;
}

uint8_t ICM42688P_init(SPI_TypeDef* spi_handle, GPIO_TypeDef* chip_select_port, uint16_t chip_select_gpio_pin)
{
    hspi = spi_handle;
    ChipSelect_GPIO_Port = chip_select_port;
    ChipSelect_Pin = chip_select_gpio_pin;

    /* // ???? WHAT??????
    HAL_Delay(100);
    ICM42688P_write_reg(0x4F, 0x04);  // Reset device
    HAL_Delay(100);
    ICM42688P_write_reg(0x11, 0x00);  // Power management
    ICM42688P_write_reg(0x10, 0x0F);  // Gyro and accel config
    */

    ICM42688P_write_reg(0x11, 0x01); // Reset Device
    HAL_Delay(100);
    ICM42688P_write_reg(0x4E, (0b11 << 2) | (0b11 << 0)); // Enable gyro & accelerometer
    ICM42688P_write_reg(0x7B, (0b10 << 1)); // Enable CLKIN

    return 0;
}

ICM42688P_AccelData ICM42688P_read_data()
{
    ICM42688P_AccelData data = { 0 };

    uint8_t buffer[12];
    ICM42688P_disable_chip_select();
    //This register should be correct
    uint8_t reg = 0x1F | (1 << 7);
    HAL_SPI_Transmit(hspi, &reg, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(hspi, buffer, sizeof(buffer) / sizeof(buffer[0]), HAL_MAX_DELAY);
    ICM42688P_enable_chip_select();

    // Transfer data to old gyro data.
    data.gyro_old_x = data.gyro_x;
    data.gyro_old_y = data.gyro_y;
    data.gyro_old_z = data.gyro_z;
    data.old_time = data.new_time;

    /*data.accel_x = (buffer[0] << 8) | buffer[1];
    data.accel_y = (buffer[2] << 8) | buffer[3];
    data.accel_z = ((buffer[4] << 8) | buffer[5])*-1;*/

    // Get new information.
    data.gyro_x = (buffer[6] << 8) | buffer[7];
    data.gyro_y = (buffer[8] << 8) | buffer[9];
    data.gyro_z = (buffer[10] << 8) | buffer[11]*-1;
    data.new_time = HAL_GetTick();

    // Calculate acceraltion from two gyro data points / time difference.
    data.accel_x = (data.gyro_x - data.gyro_old_x) / (data.new_time - data.old_time);
    data.accel_y = (data.gyro_y - data.gyro_old_y) / (data.new_time - data.old_time);
    data.accel_z = (data.gyro_z - data.gyro_old_z) / (data.new_time - data.old_time);

    return data;
}


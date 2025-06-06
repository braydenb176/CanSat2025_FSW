#include "ICM42688PSPI.h"
#include "stm32g4xx_hal.h"
#include <stdint.h>
#include "AccelerationCalculations.c"

static SPI_HandleTypeDef *hspi;

/* Private GPIO CS Pin Variables */
static GPIO_TypeDef *ChipSelect_GPIO_Port;
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

uint16_t ICM42688P_read_reg(uint8_t reg)
{
    uint8_t tx = reg | (1 << 7);
    uint8_t rx[2] = { 0 };
    ICM42688P_disable_chip_select();
    HAL_SPI_Transmit(hspi, &tx, 1, HAL_MAX_DELAY);

    HAL_SPI_Receive(hspi, &rx, 1, HAL_MAX_DELAY);
    ICM42688P_enable_chip_select();

    uint16_t shifted = rx[0] << 8;
    uint16_t lower = rx[1];
    uint16_t value = shifted | lower;
    return value;
}

uint8_t ICM42688P_init(SPI_TypeDef *spi_handle, GPIO_TypeDef *chip_select_port, uint16_t chip_select_gpio_pin)
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
    ICM42688P_write_reg(0x7B, (0b10 << 1));               // Enable CLKIN

    return 0;
}

ICM42688P_AccelData ICM42688P_read_data()
{
    ICM42688P_AccelData data = {0};

    data.accel_x = ICM42688P_read_reg(0x1F);
    data.accel_y = ICM42688P_read_reg(0x21);
    data.accel_z = ICM42688P_read_reg(0x23);
    data.gyro_x = ICM42688P_read_reg(0x25);
    data.gyro_y = ICM42688P_read_reg(0x27);
    data.gyro_z = ICM42688P_read_reg(0x29);
    
    return data;
}

int16_t gyro_old_x = 0;
int16_t gyro_old_y = 0;
int16_t gyro_old_z = 0;
int32_t old_time = 0; // Might need to reinitialized to actual start.

void Transfer_Data(uint16_t x, uint16_t y, uint16_t z, uint32_t time)
{
    gyro_old_x = x;
    gyro_old_y = y;
    gyro_old_z = z;
    old_time = time;
}

uint16_t Get_Accel_X(uint16_t gyro_x, uint32_t time)
{
    return (gyro_old_x - gyro_x) / (old_time - time);
}

uint16_t Get_Accel_Y(uint16_t gyro_y, uint32_t time)
{
    return (gyro_old_y - gyro_y) / (old_time - time);
}

uint16_t Get_Accel_Z(uint16_t gyro_z, uint32_t time)
{
    return (gyro_old_z - gyro_z) / (old_time - time);
}

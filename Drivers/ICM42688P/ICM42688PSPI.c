#include "ICM42688PSPI.h"
#include "stm32g4xx_hal.h"
#include <stdint.h>
#include "AccelerationCalculations.c"

static SPI_HandleTypeDef *hspi;

/* Private GPIO CS Pin Variables */
static GPIO_TypeDef *ChipSelect_GPIO_Port;
static uint16_t ChipSelect_Pin;


volatile static uint16_t gyro_old_r = 0;
volatile static uint16_t gyro_old_y = 0;
volatile static uint16_t gyro_old_p = 0;
volatile static uint32_t old_time = 0;

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

    data.accel_z = ICM42688P_read_reg(0x23);
    
    data.gyro_p = ICM42688P_read_reg(0x25);
    data.gyro_y = ICM42688P_read_reg(0x27);
    data.gyro_r = ICM42688P_read_reg(0x29);

    data.accel_p = Get_Accel_P(data.gyro_p, HAL_GetTick());
    data.accel_y = Get_Accel_Y(data.gyro_y, HAL_GetTick());
    data.accel_r = -Get_Accel_R(data.gyro_r, HAL_GetTick());

    gyro_old_p = data.gyro_p;
    gyro_old_y = data.gyro_y;
    gyro_old_r = data.gyro_r;
    old_time = HAL_GetTick();

    return data;
}

uint16_t Get_Accel_P(uint16_t gyro_p, uint32_t time)
{
    return (gyro_old_p - gyro_p) / (old_time - time);
}

uint16_t Get_Accel_Y(uint16_t gyro_y, uint32_t time)
{
    return (gyro_old_y - gyro_y) / (old_time - time);
}

uint16_t Get_Accel_R(uint16_t gyro_r, uint32_t time)
{
    return (gyro_old_r - gyro_r) / (old_time - time);
}

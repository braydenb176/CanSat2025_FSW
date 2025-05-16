#ifndef _ICM42688PSPI_H_
#define _ICM42688PSPI_H_

typedef struct ICM42688P_AccelData
{
    uint16_t accel_x;
    uint16_t accel_y;
    uint16_t accel_z;
    uint16_t gyro_x;
    uint16_t gyro_y;
    uint16_t gyro_z;
} ICM42688P_AccelData;

ICM42688P_AccelData ICM42688P_read_data();

uint8_t ICM42688P_init(SPI_HandleTypeDef *spi_handle, GPIO_TypeDef* chip_select_port, uint16_t chip_select_gpio_pin);

#endif

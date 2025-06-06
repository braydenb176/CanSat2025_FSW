#ifndef _ICM42688PSPI_H_
#define _ICM42688PSPI_H_

#include <stm32g491xx.h>

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif


typedef struct ICM42688P_AccelData
{
<<<<<<< Updated upstream
    uint16_t accel_x;
    uint16_t accel_y;
    uint16_t accel_z;
    uint16_t gyro_x;
    uint16_t gyro_y;
    uint16_t gyro_z;
=======
    int16_t accel_z;

    int16_t accel_p;
    int16_t accel_y;
    int16_t accel_r;

    int16_t gyro_p;
    int16_t gyro_y;
    int16_t gyro_r;
>>>>>>> Stashed changes
} ICM42688P_AccelData;

int16_t ICM42688P_read_reg(uint8_t reg);

ICM42688P_AccelData ICM42688P_read_data();

uint8_t ICM42688P_init(SPI_TypeDef *spi_handle, GPIO_TypeDef* chip_select_port, uint16_t chip_select_gpio_pin);

#endif

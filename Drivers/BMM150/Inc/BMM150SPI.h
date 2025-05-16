#ifndef _BMM150SPI_H_
#define _BMM150SPI_H_

#include "bmm150.h"

#include "stm32g4xx_hal.h"

typedef struct BMM150_dev
{
	int8_t error;
	struct bmm150_dev device;
} BMM150_dev;

typedef struct BMM150_mag_data
{
	int16_t x;
	int16_t y;
	int16_t z;
	int8_t error;
} BMM150_mag_data;

BMM150_dev BMM150_spi_init(SPI_HandleTypeDef* spi_handle, GPIO_TypeDef* cs_port, uint16_t cs_gpio_pin);

BMM150_mag_data BMM150_read_mag_data(BMM150_dev* device);

#endif

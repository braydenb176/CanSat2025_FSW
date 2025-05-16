#include "BMM150SPI.h"
#include "stm32g4xx_hal.h"

#define BMM150_TIMEOUT_DURATION 10

static SPI_HandleTypeDef* spi_handle;

static GPIO_TypeDef* chip_select_port;

static uint16_t chip_select_pin;

void BMM150_enable_chip_select()
{
	HAL_GPIO_WritePin(chip_select_port, chip_select_pin, GPIO_PIN_RESET);
}

void BMM150_disable_chip_select()
{
	HAL_GPIO_WritePin(chip_select_port, chip_select_pin, GPIO_PIN_SET);
}

void BMM150_delay(int period_in_micro_s, void* intf_ptr)
{
	//tick is approximately 1 ms
	const int tick_amount = 1000;
	osDelay(period_in_micro_s / tick_amount);
}

BMM150_INTF_RET_TYPE BMM150_SPI_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
	HAL_StatusTypeDef status = HAL_OK;

	BMM150_enable_chip_select();

	status = HAL_SPI_Transmit(spi_handle, reg_addr, sizeof(reg_addr), BMM150_TIMEOUT_DURATION);
	status = HAL_SPI_Receive(spi_handle, reg_data, length, BMM150_TIMEOUT_DURATION);

	BMM150_disable_chip_select();

	return (BMM150_INTF_RET_TYPE)status;
}

BMM150_INTF_RET_TYPE BMM150_SPI_write(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
	HAL_StatusTypeDef status = HAL_OK;

	BMM150_enable_chip_select();

	status = HAL_SPI_Transmit(spi_handle, reg_addr, sizeof(reg_addr), BMM150_TIMEOUT_DURATION);
	status = HAL_SPI_Transmit(spi_handle, reg_data, length, BMM150_TIMEOUT_DURATION);

	BMM150_disable_chip_select();

	return (BMM150_INTF_RET_TYPE)status;
}

BMM150_dev BMM150_spi_init(SPI_HandleTypeDef *spi_handle, GPIO_TypeDef* cs_port, uint16_t cs_gpio_pin)
{
	BMM150_dev bmm150 = (BMM150_dev){ 0 };

	if (spi_handle == NULL)
	{
		bmm150.error = BMM150_E_NULL_PTR;
		return bmm150;
	}

	chip_select_port = cs_port;
	chip_select_pin = cs_gpio_pin;

	bmm150.device.intf = BMM150_SPI_INTF;
	bmm150.device.read = BMM150_SPI_read;
	bmm150.device.write = BMM150_SPI_write;
	bmm150.device.delay_us = BMM150_delay;
	bmm150.device.intf_ptr = NULL;

	//NOTE: Potential error: is chip id initialized properly?
	bmm150.error = bmm150_init(&bmm150.device);

	return bmm150;
}

BMM150_mag_data BMM150_read_mag_data(BMM150_dev* bmm150)
{
	BMM150_mag_data mag_data = (BMM150_mag_data) {0};

	if (bmm150->ok)
	{
		mag_data.error = BMM150_E_NULL_PTR;
		return mag_data;
	}

	struct bmm150_mag_data data = (struct bmm150_mag_data){ 0 };

	mag_data.error = bmm150_read_mag_data(&data, bmm150->device);

	mag_data.x = data.x;
	mag_data.y = data.y;
	mag_data.z = data.z;

	return mag_data;
}

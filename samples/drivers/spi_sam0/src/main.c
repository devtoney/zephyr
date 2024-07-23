/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>

void read_id();

/**
 * @file Sample app using the Fujitsu MB85RS64V FRAM through SPI.
 */
#define SPI_NODE DT_NODELABEL(sercom1)
//#define SPI_CS_PIN 5 // Pin number as per the DTS configuration


static struct spi_cs_control spi_cs = {
    .gpio = SPI_CS_GPIOS_DT_SPEC_GET(SPI_NODE),
    .delay = 2,
};

static const struct spi_config spi_cfg = {
    .frequency = 1000000, // 1 MHz
    .operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_OP_MODE_MASTER,
    //.cs = &spi_cs,
};

const struct device *spi = DEVICE_DT_GET(SPI_NODE);

int main(void)
{

	if (!device_is_ready(spi)) {
		printk("SPI device %s is not ready\n", spi->name);
		return 0;
	}

	uint8_t tx_data[] = {0x01, 0x02, 0x03,0x04,0x05}; // Example data to send
    uint8_t rx_data[sizeof(tx_data)] = {0};

    struct spi_buf tx_buf = {
        .buf = tx_data,
        .len = sizeof(tx_data),
    };
    struct spi_buf rx_buf = {
        .buf = rx_data,
        .len = sizeof(rx_data),
    };

    struct spi_buf_set tx_bufs = {
        .buffers = &tx_buf,
        .count = 1,
    };
    struct spi_buf_set rx_bufs = {
        .buffers = &rx_buf,
        .count = 1,
    };

    read_id();

	while(1)
	{
			int ret = spi_transceive(spi, &spi_cfg, &tx_bufs, &rx_bufs);
			if (ret == 0) {
				printk("SPI transfer completed successfully\n");
				printk("Received data: ");
				for (int i = 0; i < sizeof(rx_data); i++) {
					printk("0x%02x ", rx_data[i]);
				}
				printk("\n");
			} 
			else 
			{
				printk("SPI transfer failed: %d\n", ret);
			}

			k_sleep(K_MSEC(10));
	}
	return 0;
}


 // Send Read Device ID command and read back the spi flash id 
 void read_id()
 {
	//SPI_Flash Read Device ID command and Send 3-byte address (assuming 24-bit addressing)
	uint8_t ID_Readtx_data_Commands[] = {0x90, 0x00, 0x00,0x00};
	uint8_t rx_ID_data[2] = {0};

	struct spi_buf ID_Readtx_buf = {
        .buf = ID_Readtx_data_Commands,
        .len = sizeof(ID_Readtx_data_Commands),
    };

	struct spi_buf_set ID_Readtx_bufs = {
        .buffers = &ID_Readtx_buf,
        .count = 1,
    };

	struct spi_buf ID_Readrx_buf = {
        .buf = rx_ID_data,
        .len = sizeof(rx_ID_data),
    };

	struct spi_buf_set ID_Readrx_bufs = {
        .buffers = &ID_Readrx_buf,
        .count = 1,
    };

	int ret = spi_transceive(spi, &spi_cfg, &ID_Readtx_bufs, &ID_Readrx_buf);

	if (ret == 0) {
		printk("SPI transfer completed successfully\n");
		printk("Received device ID data: ");
		for (int i = 0; i < sizeof(ID_Readrx_buf); i++) {
			printk("0x%02x ", rx_ID_data[i]);
		}
		printk("\n");
	} 
	else 
    {
		printk("Failed to read SPI device ID: %d\n", ret);
	}
 }
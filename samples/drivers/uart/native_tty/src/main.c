/*
 * @brief Native TTY UART sample
 *
 * Copyright (c) 2023 Marko Sagadin
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
// #include "C:/Users/emira/toney/zephyrproject/modules/hal/atmel/asf/sam0/include/samd11/samd11d14am.h"
// #include "C:/Users/emira/toney/zephyrproject/modules/hal/atmel/asf/sam0/include/samd11/component/sercom.h"
// #include "C:/Users/emira/toney/zephyrproject/modules/hal/atmel/asf/sam0/include/samd11/component/port.h"
#include <stdio.h>
#include <string.h>
unsigned char send_buf[20] = "Hello from device ";
unsigned char recv_buf[10];

const struct device *uart0 = DEVICE_DT_GET(DT_NODELABEL(sercom0));

struct uart_config uart_cfg = {
	.baudrate = 115200,
	.parity = UART_CFG_PARITY_NONE,
	.stop_bits = UART_CFG_STOP_BITS_1,
	.flow_ctrl = UART_CFG_FLOW_CTRL_NONE,
	.data_bits = UART_CFG_DATA_BITS_8,
};

void send_str(const struct device *uart, char *str)
{
	int msg_len = strlen(str);

	for (int i = 0; i < msg_len; i++) {

		uart_poll_out(uart, str[i]);
	}

	//printk("Device %s sent: \"%s\"\n", uart->name, str);
}

void recv_str(const struct device *uart, char *str)
{
	char *head = str;
	char c;
	while (!uart_poll_in(uart, &c)) {
		*head++ = c;
	}
	*head = '\0';

	//printk("Device %s received: \"%s\"\n", uart->name, str);
}

int main(void)
{
	int rc;
	
	if (!device_is_ready(uart0)) {
		printk("UART device %s not ready\n", uart0->name);
		return -1;
	}
	rc = uart_configure(uart0, &uart_cfg);
	if (rc) {
		printk("Could not configure device %s", uart0->name);
	}

	while (1) {
		   recv_str(uart0, recv_buf);
		   //snprintf(send_buf, 64, "Hello from device %s", uart0->name);
		   //int res = strcmp(recv_buf,"hello");
		   if(recv_buf[0] == 'h')
		   {
	            send_str(uart0, send_buf);
		   }
		   k_sleep(K_MSEC(1000));
		   
	}

	return 0;
}

/*
 * Copyright (c) 2024 Centro de Inovacao EDGE
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/adc.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>

#define ADC_CHANNEL_ID  8
#define ADC_RESOLUTION  10
#define BUFFER_SIZE     1

/* Array to collect stor adc result . */
static uint16_t sample_buffer[BUFFER_SIZE];
/* ADC node from the devicetree. */
#define ADC_NODE DT_NODELABEL(adc)

/* Data of ADC device specified in devicetree. */
static const struct device *adc_dev = DEVICE_DT_GET(ADC_NODE);

/* Data array of ADC channels for the specified ADC. */
static const struct adc_channel_cfg channel_cfgs = {
	.gain = ADC_GAIN_1,
    .reference = ADC_REF_INTERNAL,
    .acquisition_time = ADC_ACQ_TIME_DEFAULT,
    .channel_id = ADC_CHANNEL_ID,
    //.input_positive = ADC_CHANNEL_ID,
};

struct adc_sequence sequence = {
        .channels = BIT(ADC_CHANNEL_ID),
        .buffer = sample_buffer,
        .buffer_size = sizeof(sample_buffer),
        .resolution = ADC_RESOLUTION,
};



int main(void)
{
	int ret;

    if (!device_is_ready(adc_dev)) {
        printk("ADC device not found\n");
        return 0;
    }

	/* Configure channel prior to sampling. */
	ret = adc_channel_setup(adc_dev, &channel_cfgs);
	if (ret < 0) {
	    printf("Could not setup channel #%d \n", ret);
		return 0;
		}

	while (1) {
        ret = adc_read(adc_dev, &sequence);
        if (ret) {
            printk("ADC read failed: %d\n", ret);
        } else {
            printk("ADC raw value: %u\n", sample_buffer[0]);
        }
        k_sleep(K_MSEC(1000));
    }
}




// #include <inttypes.h>
// #include <stddef.h>
// #include <stdint.h>

// #include <zephyr/device.h>
// #include <zephyr/devicetree.h>
// #include <zephyr/drivers/adc.h>
// #include <zephyr/kernel.h>
// #include <zephyr/sys/printk.h>
// #include <zephyr/sys/util.h>

// #define SLEEP_TIME_MS   1000
// #define RESOLUTION 12

// static const struct adc_dt_spec adc_channel8 =  ADC_DT_SPEC_GET_BY_IDX(DT_PATH(adc), 0);

// int16_t buffer,buff;
// // Define ADC sequences for each channel

//  struct adc_sequence sequence_ch0 = {
//     .channels = BIT(8),
//     .buffer = &buff,
//     .buffer_size = sizeof(buff),
//     .resolution = RESOLUTION,
// };



// int main(void)
// {
//     int err;

//     if (!adc_is_ready_dt(&adc_channel8)) {
// 	printk("ADC controller devivce %s not ready", adc_channel8.dev->name);
// 	return 0;
// }

//  while(1){

//     err = adc_channel_setup_dt(&adc_channel8);
//     if (err < 0) {
// 	printk("Could not setup channel #%d (%d)", 0, err);
// 	return 0;
// }

//     err = adc_sequence_init_dt(&adc_channel8, &sequence_ch0);
// 	if (err < 0) {
// 		printk("Could not initalize sequnce");
// 		return 0;
// 	}

//     err = adc_read_dt(&adc_channel8, &sequence_ch0);
// 		if (err < 0) {
// 			printk("Could not read (%d)", err);
			
// 		}

//     int32_t mv_value_ch0 =(int32_t)((int16_t) buff);
//      printk("channel : 8 :");
//     printk("%"PRId32, mv_value_ch0);
//     printk("\n");
//     err = adc_raw_to_millivolts_dt(&adc_channel8, &mv_value_ch0);

//     k_sleep(K_MSEC(SLEEP_TIME_MS));
//  }
// }

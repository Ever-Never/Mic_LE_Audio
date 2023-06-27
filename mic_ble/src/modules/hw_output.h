
#ifndef _HARDWARE_OUTPUT_
#define _HARDWARE_OUTPUT_

#include <stdint.h>
#include <stdbool.h>
typedef enum
{
    GPIO_MIC_OUT = 0,
    GPIO_ON_OFF_CTRL,
}gpio_output_t;
int hw_output_init();

int hw_output_set_level(gpio_output_t output_pin, int level);

bool hw_output_get_level(gpio_output_t output_pin);
void hw_output_button_handle();
#endif

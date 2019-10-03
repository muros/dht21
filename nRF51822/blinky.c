#include <stdbool.h>
#include <stdint.h>
#include "nrf_gpio.h"
#include "nrf_delay.h"



int main(void)
{
/**
 * @brief Function for application main entry.
 */
 // setup
    // Configure LED-pin as outputs and clear.
    nrf_gpio_cfg_output(19);
    nrf_gpio_pin_clear(19);
    nrf_gpio_pin_set(19);
    nrf_gpio_cfg_output(22);
    nrf_gpio_pin_clear(22);

    for (;;)
        {
            nrf_gpio_pin_toggle(19);
            nrf_delay_ms(1000);
        }
}

/** @} */

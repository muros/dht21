/*
 * timers.c
 *
 *  Created on: Dec 5, 2017
 *      Author: uros
 */
#include "app_timer.h"
#include "nrf_gpio.h"
#include "nrf_soc.h"
#include "dht_service.h"
#include "timers.h"
#include "dht.h"

APP_TIMER_DEF(m_dht_timer_id);

int dhtTemp;
int dhtHmdt;

extern ble_os_t m_dht_service;

/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
void timers_init()
{
    // Initialize timer module.
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);
}

// Create timers
void create_timers()
{
    uint32_t err_code;

    // Create timers
    err_code = app_timer_create(&m_dht_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                timer_dht_handler);
    APP_ERROR_CHECK(err_code);
}

void start_timers()
{
	uint32_t err_code;

	err_code = app_timer_start(m_dht_timer_id, APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER), NULL);
	APP_ERROR_CHECK(err_code);
}

void stop_timers()
{
	uint32_t err_code;

	err_code = app_timer_stop(m_dht_timer_id);
	APP_ERROR_CHECK(err_code);
}

// Timeout handler for the repeated timer
void timer_dht_handler(void * p_context)
{
	int32_t temperature = 0;
	int32_t humidity = 0;

	stop_timers();
	sample();
	start_timers();
	dhtTemp = dhtReadTemperature();
	dhtHmdt = dhtReadHumidity();
	temperature = dhtTemp;
	humidity = dhtHmdt;

	dht_th_characteristic_update(&m_dht_service, &temperature, &humidity);

}


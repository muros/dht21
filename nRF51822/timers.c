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
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "nrf_delay.h"

APP_TIMER_DEF(m_dht_timer_id);
APP_TIMER_DEF(m_ble_timer_id);

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

    err_code = app_timer_create(&m_ble_timer_id,
                                    APP_TIMER_MODE_REPEATED,
                                    timer_ble_handler);
    APP_ERROR_CHECK(err_code);
    err_code = app_timer_create(&m_dht_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                timer_dht_handler);
    APP_ERROR_CHECK(err_code);
}

void start_timers() {
    start_timer_dht();
}

void start_timer_dht() {
    uint32_t err_code;

    err_code = app_timer_start(m_dht_timer_id,
            APP_TIMER_TICKS(60000, APP_TIMER_PRESCALER), NULL);
    APP_ERROR_CHECK(err_code);
}

void stop_timer_dht() {
    uint32_t err_code;

    err_code = app_timer_stop(m_dht_timer_id);
    if (err_code != NRF_SUCCESS) {
        nrf_gpio_pin_set(22);
    }
    APP_ERROR_CHECK(err_code);
}

void start_timer_ble() {
    uint32_t err_code;

    err_code = app_timer_start(m_ble_timer_id,
            APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER), NULL);
    APP_ERROR_CHECK(err_code);
}

void stop_timer_ble() {
    uint32_t err_code;

    err_code = app_timer_stop(m_ble_timer_id);
    APP_ERROR_CHECK(err_code);
}

void stop_timers() {
    uint32_t err_code;

    err_code = app_timer_stop(m_dht_timer_id);
    APP_ERROR_CHECK(err_code);
    err_code = app_timer_stop(m_ble_timer_id);
    APP_ERROR_CHECK(err_code);
}

// Timeout handler for the repeated read of DHT sensor
void timer_dht_handler(void * p_context) {
    nrf_gpio_pin_toggle(19);

    sample();
}

// Timeout handler for the repeated update of BLE advertising data
void timer_ble_handler(void * p_context)
{
    nrf_gpio_pin_toggle(22);
    unsigned char dhtData[5];

    dhtData[0] = dhtData[1] = dhtData[2] = dhtData[3] = dhtData[4] = 0x12;

    // Reading and calculating data form DHT22
    //dhtTemp = dhtReadTemperature();
    //dhtHmdt = dhtReadHumidity();
    //temperature = dhtTemp;
    //humidity = dhtHmdt;

    // Reading raw byte data from DHT22
    dhtReadRaw(dhtData);

    // On die thermometer
    //temperature_data_get(data);

    //dht_th_characteristic_update(&m_dht_service, &temperature, &humidity);
    //dht_th_characteristic_update_raw(&m_dht_service, data);

    // advertise data update
    advertising_update(dhtData);
}

void advertising_update(unsigned char *outData) {
    uint32_t err_code;
    uint8_t datax[21] = { 0x02, 0x01, 0x06, 0x07, 0xff, 0x59, 0x00, outData[0],
            outData[1], outData[2], outData[3], 0x09, 0x09, 0x44, 0x48, 0x54,
            0x32, 0x31, 0x2d, 0x76, 0x32 };

    err_code = sd_ble_gap_adv_data_set(datax, 21, NULL, 0);
    APP_ERROR_CHECK(err_code);
}

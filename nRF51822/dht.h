/*
 * dht.h
 *
 *  Created on: Nov 18, 2017
 *      Author: uros
 */

#ifndef DHT_H_
#define DHT_H_

// Define types of sensors.
#define DHT11 11
#define DHT22 22
#define DHT21 21
#define AM2301 21

// Define states
#define LOW false
#define HIGH true

#include "softdevice_handler.h"

unsigned char* dhtInit(unsigned char pin, unsigned char type, unsigned char count);

void dhtBegin(void);

void sample(void);

int dhtReadTemperature(void);

int dhtReadHumidity(void);

bool dhtRead();

void dhtReadRaw(unsigned char *outData);

void temperature_data_get(unsigned char *outData);

uint32_t dhtExpectPulse(bool level, uint32_t maxWait);

void timeCriticalDHTread();

void publishReadData();

// ********************************************************************
// * Timeslot specific code
// * for providing time dependent interrupt-less functionality.
// ********************************************************************

/**@brief Request next timeslot event in earliest configuration
 */
uint32_t request_next_event_earliest_dht(void);

/**@brief Configure next timeslot event in earliest configuration
 */
void configure_next_event_earliest_dht(void);

/**@brief Timeslot signal handler
 */
void nrf_evt_signal_handler_dht(uint32_t evt_id);


/**@brief Timeslot event handler
 */
nrf_radio_signal_callback_return_param_t * radio_callback_dht(uint8_t signal_type);

/**@brief Function for initializing the timeslot API.
 */
uint32_t timeslot_sd_init_dht(void);

/**@brief Function for finalizing the timeslot API.
 */
uint32_t timeslot_sd_finalize_dht(void);

#endif /* DHT_H_ */

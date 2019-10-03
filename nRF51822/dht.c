/*
 * dht.c
 *
 *  Created on: Nov 18, 2017
 *      Author: uros
 */

#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include "nrf.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "app_timer.h"
#include "dht.h"
#include "app_uart.h"
#include "app_error.h"


//#include "uart.h"

#define MIN_INTERVAL 2000

#define MAX_SAMPLE_TRY 1

unsigned char dhtPin;

unsigned char dhtType;

unsigned char data[5];

int temp;

int hmdt;

// Success of last read of DHT sensor data
bool _lastresult;

// Asynchronous DHT read
bool readInProgress;

/**Constants for timeslot API
*/
static nrf_radio_request_t  m_timeslot_request;
static uint32_t             m_slot_length;

static nrf_radio_signal_callback_return_param_t signal_callback_return_param;


unsigned char* dhtInit(unsigned char pin, unsigned char type, unsigned char count)
{
	dhtPin = pin;
	dhtType = type;
	data[0] = data[1] = data[2] = data[3] = data[4] = 0x13;

	return data;
}

void dhtBegin(void)
{
	nrf_gpio_cfg_input(dhtPin, NRF_GPIO_PIN_PULLUP);
}

void sample(void) {
    bool success = false;
    success = dhtRead();
    if (success)
    {
        switch (dhtType) {
        case DHT11:
            temp = data[2];
            break;
        case DHT22:
        case DHT21:
            temp = data[2] & 0x7F;
            temp *= 256;
            temp += data[3];
            //f *= 0.1;
            if (data[2] & 0x80) {
                temp *= -1;
            }
            break;
        }
        switch (dhtType) {
        case DHT11:
            hmdt = data[0];
            break;
        case DHT22:
        case DHT21:
            hmdt = data[0];
            hmdt *= 256;
            hmdt += data[1];
            //f *= 0.1;
            break;
        }
    }
    else
    {
        // Bad sample
        data[0] = data[1] = data[2] = data[3] = 0x00;
        data[4] = 0xff;
    }
}

int dhtReadTemperature()
{
    return temp;
}

int dhtReadHumidity()
{
    return hmdt;
}

void dhtReadRaw(unsigned char *outData)
{
    outData[0] = data[0];
    outData[1] = data[1];
    outData[2] = data[2];
    outData[3] = data[3];
    outData[4] = data[4];
}

void temperature_data_get(unsigned char *outData)
{
    int32_t temp;
    uint32_t err_code;

    err_code = sd_temp_get(&temp);
    APP_ERROR_CHECK(err_code);

    temp = (temp / 4) * 100;

    int8_t exponent = -2;
    uint32_t retTemp;
	retTemp = ((exponent & 0xFF) << 24) | (temp & 0x00FFFFFF);
	outData[0] = retTemp & 0xFF;
	outData[1] = (retTemp >> 8) & 0xFF;
	outData[2] = (retTemp >> 16) & 0xFF;
	outData[3] = (retTemp >> 24) & 0xFF;
	outData[4] = 0x00;
}

bool dhtRead() {
    // Reset 40 bits of received data to zero.
    data[0] = data[1] = data[2] = data[3] = data[4] = 0x00;
    // Send start signal.  See DHT datasheet for full signal diagram:
    //   http://www.adafruit.com/datasheets/Digital%20humidity%20and%20temperature%20sensor%20AM2302.pdf

    // Go into high impedance state to let pull-up raise data line level and
    // start the reading process.
    nrf_gpio_pin_set(dhtPin);
    nrf_delay_ms(250);

    // First set data line low for 20 milliseconds.
    nrf_gpio_cfg_output(dhtPin);
    nrf_gpio_pin_clear(dhtPin);
    nrf_delay_ms(20);

    readInProgress = true;
/*
    timeslot_sd_init_dht();
    // Read of DHT data is done asynchronously in timeslot.
    while (readInProgress) {
        //nrf_delay_ms(1);
    }
    timeslot_sd_finalize_dht();
*/
    timeCriticalDHTread();
    publishReadData();


    return _lastresult;
}
/**@brief Time critical section of code that reads response from DHT
 * sensor and should not be interrupted.
 */
void timeCriticalDHTread() {
    uint32_t cycles[80];
    {
        // *** Time critical section START ***

        // End the start signal by setting data line high for 40 microseconds.
        //nrf_gpio_cfg_input(dhtPin, NRF_GPIO_PIN_PULLUP);
        nrf_gpio_pin_set(dhtPin);
        nrf_delay_us(40);

        // Now start reading the data line to get the value from the DHT sensor.
        nrf_gpio_cfg_input(dhtPin, NRF_GPIO_PIN_PULLUP);
        nrf_delay_us(10);

        // First expect a low signal for ~80 microseconds followed by a high signal
        // for ~80 microseconds again.
        if (dhtExpectPulse(LOW, 50) == 0) {
            _lastresult = false;
            readInProgress = false;
            return;
        }
        if (dhtExpectPulse(HIGH, 80) == 0) {
            _lastresult = false;
            readInProgress = false;
            return;
        }
        // Now read the 40 bits sent by the sensor.  Each bit is sent as a 50
        // microsecond low pulse followed by a variable length high pulse.  If the
        // high pulse is ~28 microseconds then it's a 0 and if it's ~70 microseconds
        // then it's a 1.  We measure the cycle count of the initial 50us low pulse
        // and use that to compare to the cycle count of the high pulse to determine
        // if the bit is a 0 (high state cycle count < low state cycle count), or a
        // 1 (high state cycle count > low state cycle count). Note that for speed all
        // the pulses are read into a array and then examined in a later step.
        for (int i = 0; i < 80; i += 2) {
            cycles[i] = dhtExpectPulse(LOW, 50);
            cycles[i + 1] = dhtExpectPulse(HIGH, 80);
        }

        // *** Time critical section END ***
    }

    // Inspect pulses and determine which ones are 0 (high state cycle count < low
    // state cycle count), or 1 (high state cycle count > low state cycle count).
    for (int i = 0; i < 40; ++i) {
        uint32_t lowCycles = cycles[2 * i];
        uint32_t highCycles = cycles[2 * i + 1];
        if ((lowCycles == 0) || (highCycles == 0)) {
            _lastresult = false;
            readInProgress = false;
            return;
        }
        data[i / 8] <<= 1;
        // Now compare the low and high cycle times to see if the bit is a 0 or 1.
        if (highCycles > lowCycles) {
            // High cycles are greater than 50us low cycle count, must be a 1.
            data[i / 8] |= 1;
        }
        // Else high cycles are less than (or equal to, a weird case) the 50us low
        // cycle count so this must be a zero.  Nothing needs to be changed in the
        // stored data.
    }
    // Check we read 40 bits and that the checksum matches.
    if (data[4] == ((data[0] + data[1] + data[2] + data[3]) & 0xFF)) {
        _lastresult = true;
        readInProgress = false;
    } else {
        _lastresult = false;
        readInProgress = false;
    }
}

void publishReadData() {
    uint32_t err_code;
    uint8_t datax[22] = { 0x02, 0x01, 0x06, 0x08, 0xff, 0x59, 0x00, 0x00, 0x00,
            0x00, 0x00, 0xaa, 0x09, 0x09, 0x44, 0x48, 0x54, 0x32, 0x31, 0x2d,
            0x76, 0x32 };

    datax[7] = data[0];
    datax[8] = data[1];
    datax[9] = data[2];
    datax[10] = data[3];
    datax[11] = data[4];
    err_code = sd_ble_gap_adv_data_set(datax, 22, NULL, 0);
    APP_ERROR_CHECK(err_code);
}

// Expect the signal line to be at the specified level for a period of time and
// return a count of loop cycles spent at that level (this cycle count can be
// used to compare the relative time of two pulses).  If more than a millisecond
// elapses without the level changing then the call fails with a 0 response.
// This is adapted from Arduino's pulseInLong function (which is only available
// in the very latest IDE versions):
//   https://github.com/arduino/Arduino/blob/master/hardware/arduino/avr/cores/arduino/wiring_pulse.c
uint32_t dhtExpectPulse(bool level, uint32_t maxWait) {
	uint32_t hold = 0;

	while (nrf_gpio_pin_read(dhtPin) == level) {
		nrf_delay_us(1);
		hold += 1;
		if (hold > maxWait) {
			return 0; // Exceeded timeout, fail.
		}
	}

	return hold;
}

// ********************************************************************
// * Timeslot specific code
// * for providing time dependent interrupt-less functionality.
// ********************************************************************

/**@brief Configure next timeslot event in earliest configuration
 */
void configure_next_event_earliest_dht(void)
{
    m_slot_length                                  = 8000;
    m_timeslot_request.request_type                = NRF_RADIO_REQ_TYPE_EARLIEST;
    m_timeslot_request.params.earliest.hfclk       = NRF_RADIO_HFCLK_CFG_XTAL_GUARANTEED;
    m_timeslot_request.params.earliest.priority    = NRF_RADIO_PRIORITY_HIGH;
    m_timeslot_request.params.earliest.length_us   = m_slot_length;
    m_timeslot_request.params.earliest.timeout_us  = 3000; // original 3s
}

/**@brief Request next timeslot event in earliest configuration
 */
uint32_t request_next_event_earliest_dht(void)
{
    m_slot_length                                  = 8000;
    m_timeslot_request.request_type                = NRF_RADIO_REQ_TYPE_EARLIEST;
    m_timeslot_request.params.earliest.hfclk       = NRF_RADIO_HFCLK_CFG_XTAL_GUARANTEED;
    m_timeslot_request.params.earliest.priority    = NRF_RADIO_PRIORITY_HIGH;
    m_timeslot_request.params.earliest.length_us   = m_slot_length;
    m_timeslot_request.params.earliest.timeout_us  = 3000; // original 3s
    return sd_radio_request(&m_timeslot_request);
}

/**@brief Timeslot signal handler
 */
void nrf_evt_signal_handler_dht(uint32_t evt_id)
{
    uint32_t err_code;

    switch (evt_id)
    {
        case NRF_EVT_RADIO_SIGNAL_CALLBACK_INVALID_RETURN:
            //No implementation needed
            break;
        case NRF_EVT_RADIO_SESSION_IDLE:
            //No implementation needed
            break;
        case NRF_EVT_RADIO_SESSION_CLOSED:
            //No implementation needed, session ended
            publishReadData();
            break;
        case NRF_EVT_RADIO_BLOCKED:
            //Fall through
        case NRF_EVT_RADIO_CANCELED:
            err_code = request_next_event_earliest_dht();
            APP_ERROR_CHECK(err_code);
            break;
        default:
            break;
    }
}

/**@brief Timeslot event handler
 */
nrf_radio_signal_callback_return_param_t * radio_callback_dht(uint8_t signal_type)
{
    switch(signal_type)
    {
        case NRF_RADIO_CALLBACK_SIGNAL_TYPE_START:
            //Start of the timeslot - set up timer interrupt
            signal_callback_return_param.params.request.p_next = NULL;
            signal_callback_return_param.callback_action = NRF_RADIO_SIGNAL_CALLBACK_ACTION_NONE;

            //FIXME Če vržem timer ven pomaga!!!
            NRF_TIMER0->INTENSET = TIMER_INTENSET_COMPARE0_Msk;
            NRF_TIMER0->CC[0] = m_slot_length - 1000;
//            sd_nvic_SetPriority(TIMER0_IRQn, 3);
//            sd_nvic_EnableIRQ(TIMER0_IRQn);
            //NVIC_EnableIRQ(TIMER0_IRQn);

            // Implementation time sensitive DHT sensor read
            timeCriticalDHTread();
            //nrf_gpio_pin_toggle(22);
            break;

        case NRF_RADIO_CALLBACK_SIGNAL_TYPE_RADIO:
            signal_callback_return_param.params.request.p_next = NULL;
            signal_callback_return_param.callback_action = NRF_RADIO_SIGNAL_CALLBACK_ACTION_NONE;
            break;

        case NRF_RADIO_CALLBACK_SIGNAL_TYPE_TIMER0:
            //Timer interrupt - do graceful shutdown - is something to do on dht data or if dht was not successful?
        	signal_callback_return_param.params.request.p_next = NULL;
        	signal_callback_return_param.callback_action = NRF_RADIO_SIGNAL_CALLBACK_ACTION_NONE;
            break;
        case NRF_RADIO_CALLBACK_SIGNAL_TYPE_EXTEND_SUCCEEDED:
            //No implementation needed
            break;
        case NRF_RADIO_CALLBACK_SIGNAL_TYPE_EXTEND_FAILED:
            //Try scheduling a new timeslot
        	// We don't use extend right now. It will not happen.
            configure_next_event_earliest_dht();
            signal_callback_return_param.params.request.p_next = &m_timeslot_request;
            signal_callback_return_param.callback_action = NRF_RADIO_SIGNAL_CALLBACK_ACTION_REQUEST_AND_END;
            break;
        default:
            //No implementation needed
            break;
    }
    return (&signal_callback_return_param);
}


/**@brief Function for initializing the timeslot API.
 */
uint32_t timeslot_sd_init_dht(void)
{
    uint32_t err_code;

    err_code = sd_radio_session_open(radio_callback_dht);
    if (err_code != NRF_SUCCESS)
    {
    	readInProgress = false;
        return err_code;
    }

    err_code = request_next_event_earliest_dht();
    if (err_code != NRF_SUCCESS)
    {
        (void)sd_radio_session_close();
        readInProgress = false;
        return err_code;
    }
    return NRF_SUCCESS;
}

/**@brief Function for finalizing the timeslot API.
 */
uint32_t timeslot_sd_finalize_dht(void)
{
    uint32_t err_code;

    err_code = sd_radio_session_close();
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    return NRF_SUCCESS;
}

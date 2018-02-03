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

unsigned char dhtPin;

unsigned char dhtType;

unsigned char data[5];

int temp;

int hmdt;

bool _lastresult;

void dhtInit(unsigned char pin, unsigned char type, unsigned char count)
{
	dhtPin = pin;
	dhtType = type;
}

void dhtBegin(void)
{
	nrf_gpio_cfg_input(dhtPin, NRF_GPIO_PIN_PULLUP);
}

void sample(void)
{
	if (dhtRead())
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

bool dhtRead()
{
	// Check if sensor was read less than two seconds ago and return early
	// to use last reading.
//	uint32_t currenttime = millis();
//	if ((currenttime - _lastreadtime) < 2000) {
//		return _lastresult; // return last correct measurement
//	}
//	_lastreadtime = currenttime;

	// Reset 40 bits of received data to zero.
	data[0] = data[1] = data[2] = data[3] = data[4] = 0;

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

	uint32_t cycles[80];
	{
		// Turn off interrupts temporarily because the next sections are timing critical
		// and we don't want any interruptions.
		//InterruptLock lock;
//		stop_timer();

		// End the start signal by setting data line high for 40 microseconds.
		nrf_gpio_cfg_input(dhtPin, NRF_GPIO_PIN_PULLUP);
		nrf_gpio_pin_set(dhtPin);
		nrf_delay_us(40);

		// Now start reading the data line to get the value from the DHT sensor.
		//delayMicroseconds(10);  // Delay a bit to let sensor pull data line low.
		nrf_delay_us(10);

		// First expect a low signal for ~80 microseconds followed by a high signal
		// for ~80 microseconds again.
		if (dhtExpectPulse(LOW, 80) == 0) {
			//DEBUG_PRINTLN(F("Timeout waiting for start signal low pulse."));
			//uart_printf("Timeout waiting for start signal low pulse.\r\n");
			_lastresult = false;
			return _lastresult;
		}
		if (dhtExpectPulse(HIGH, 80) == 0) {
			//DEBUG_PRINTLN(F("Timeout waiting for start signal high pulse."));
			//uart_printf("Timeout waiting for start signal high pulse.\r\n");
			_lastresult = false;
			return _lastresult;
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
			cycles[i] = dhtExpectPulse(LOW, 70);
			cycles[i + 1] = dhtExpectPulse(HIGH, 80);
		}

//		start_timer();
	} // Timing critical code is now complete.

	// Inspect pulses and determine which ones are 0 (high state cycle count < low
	// state cycle count), or 1 (high state cycle count > low state cycle count).
	for (int i = 0; i < 40; ++i) {
		uint32_t lowCycles = cycles[2 * i];
		uint32_t highCycles = cycles[2 * i + 1];
		if ((lowCycles == 0) || (highCycles == 0)) {
			//DEBUG_PRINTLN(F("Timeout waiting for pulse."));
			//uart_printf("Timeout waiting for pulse.\r\n");
			_lastresult = false;
			return _lastresult;
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

//	uart_printf("Received:\r\n");
//	uart_printf("%d\r\n", data[0]);
//	uart_printf("%d\r\n", data[1]);
//	uart_printf("%d\r\n", data[2]);
//	uart_printf("%d\r\n", data[3]);
//	uart_printf("%d\r\n", data[4]);
//	uart_printf("%d\r\n", (data[0] + data[1] + data[2] + data[3]) & 0xFF);

	// Check we read 40 bits and that the checksum matches.
	if (data[4] == ((data[0] + data[1] + data[2] + data[3]) & 0xFF)) {
		_lastresult = true;
//		uart_printf("Checksum OK.\r\n");
		return _lastresult;
	} else {
		//DEBUG_PRINTLN(F("Checksum failure!"));
		//uart_printf("Checksum failure.\r\n");
		_lastresult = false;
		return _lastresult;
	}
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
		nrf_delay_us(10);
		hold += 10;
		if (hold > maxWait) {
			return 0; // Exceeded timeout, fail.
		}
	}

	return hold;
}

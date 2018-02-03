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

void dhtInit(unsigned char pin, unsigned char type, unsigned char count);

void dhtBegin(void);

void sample(void);

int dhtReadTemperature(void);

int dhtReadHumidity(void);

bool dhtRead();

uint32_t dhtExpectPulse(bool level, uint32_t maxWait);

#endif /* DHT_H_ */

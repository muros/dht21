/*
 * timers.h
 *
 *  Created on: Dec 5, 2017
 *      Author: uros
 */

#ifndef TIMERS_H_
#define TIMERS_H_

#define APP_TIMER_PRESCALER              0                                          /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_OP_QUEUE_SIZE          4                                          /**< Size of timer operation queues. */

void timers_init();

void create_timers();

void start_timers();

void stop_timers();

void timer_dht_handler(void * p_context);

#endif /* TIMERS_H_ */

/*
 * dht_service.h
 *
 *  Created on: Dec 3, 2017
 *      Author: uros
 */

#ifndef DHT_SERVICE_H_
#define DHT_SERVICE_H_

#include <stdint.h>
#include "ble.h"
#include "ble_srv_common.h"

// Defining 16-bit service and 128-bit base UUIDs
#define BLE_UUID_DHT_BASE_UUID     {{0x23, 0xD1, 0x13, 0xEF, 0x5F, 0x78, 0x23, 0x15, 0xDE, 0xEF, 0x12, 0x12, 0x00, 0x00, 0x00, 0x00}} // 128-bit base UUID
#define BLE_UUID_DHT_SERVICE_UUID  0xF00D // Just a random, but recognizable value

// Defining 16-bit characteristic UUID
#define BLE_UUID_DHT_CHARACTERISTC_UUID  0xBEEF // Just a random, but recognizable value

// This structure contains various status information for our service.
// The name is based on the naming convention used in Nordics SDKs.
// 'ble� indicates that it is a Bluetooth Low Energy relevant structure and
// �os� is short for Our Service).
typedef struct
{
    uint16_t                    conn_handle;    /**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection).*/
    uint16_t                    service_handle; /**< Handle of Our Service (as provided by the BLE stack). */
    // Add handles for the characteristic attributes to our struct
    ble_gatts_char_handles_t    char_handles;
    int prev_temp;
    int prev_hmdt;
    unsigned char data[5];

}ble_os_t;

/**@brief Function for handling BLE Stack events related to our service and characteristic.
 *
 * @details Handles all events from the BLE stack of interest to Our Service.
 *
 * @param[in]   p_our_service       Our Service structure.
 * @param[in]   p_ble_evt  Event received from the BLE stack.
 */
void ble_dht_service_on_ble_evt(ble_os_t * p_our_service, ble_evt_t * p_ble_evt);

/**@brief Function for initializing our new service.
 *
 * @param[in]   p_our_service       Pointer to Our Service structure.
 */
void dht_service_init(ble_os_t * p_our_service);

/** @brief Function to be called when updating and sending characteristic value.
 *
 * @details BLE characteristic is updated if change of temperature or humidity is
 * big enough. This solves issue with value being updated because inacurate
 * measurement.
 *
 * @param[in] p_dht_service structure holding dht service
 * @param[in] p_temp_value pointer to new measured temperature value
 * @param[in] p_hmdt_value pointer to new measured humidity value
 */
void dht_th_characteristic_update(ble_os_t *p_dht_service, int32_t *p_temp_value, int32_t *p_hmdt_value);

void dht_th_characteristic_update_raw(ble_os_t *p_dht_service, unsigned char *data);

#endif /* DHT_SERVICE_H_ */

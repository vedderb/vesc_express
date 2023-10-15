/*
	Copyright 2023 Rasmus Söderhielm    rasmus.soderhielm@gmail.com

	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	The VESC firmware is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program.  If not, see <http://www.gnu.org/licenses/>.
	*/

#ifndef MAIN_BLE_CUSTOM_BLE_H_
#define MAIN_BLE_CUSTOM_BLE_H_

#include "esp_bt_defs.h"
#include "esp_gatt_defs.h"

#define CUSTOM_BLE_MAX_NAME_LEN 30

typedef enum {
	CUSTOM_BLE_OK                     = 0,
	/** Generic error. (Don't think this is ever returned to an external
	   caller...) */
	CUSTOM_BLE_ERROR                  = 1,
	/** Internal error caused by some ESP function. */
	CUSTOM_BLE_ESP_ERROR              = 2,
	/** Represents some internal error. Should hopefully never be returned. */
	CUSTOM_BLE_INTERNAL_ERROR         = 3,
	/** Failed due to the BLE server beeing started. */
	CUSTOM_BLE_ALREADY_STARTED        = 4,
	/** Failed due to the BLE server having not yet been started. */
	CUSTOM_BLE_NOT_STARTED            = 5,
	/** The name given to custom_ble_set_name was too long. */
	CUSTOM_BLE_NAME_TOO_LONG          = 6,
	/** Tried to create more than the configured amount of services. */
	CUSTOM_BLE_TOO_MANY_SERVICES      = 7,
	/** Tried to create more characteristics and/or descriptors than the
	   configured capacity. */
	CUSTOM_BLE_TOO_MANY_CHR_AND_DESCR = 8,
	/** The previously ran init function failed, causing this function to fail.
	 */
	CUSTOM_BLE_INIT_FAILED            = 9,
	/** Waiting for the BLE server to process action timed out. */
	CUSTOM_BLE_TIMEOUT                = 10,
	/** The specified server, characteristic, or descriptor handle didn't exist.
	 */
	CUSTOM_BLE_INVALID_HANDLE         = 11,
	/** Tried to remove a service which was not the most recent (not aldready
	 * removed) one. */
	CUSTOM_BLE_SERVICE_NOT_LAST       = 12,
} custom_ble_result_t;

typedef struct {
	esp_bt_uuid_t uuid;
	esp_gatt_perm_t perm;

	uint16_t value_max_len;
	uint16_t value_len;
	/**
	 * The initial value. A copy of this value is created during
	 * initialization. May be null.
	 */
	uint8_t *value;
} ble_desc_definition_t;

typedef struct {
	esp_bt_uuid_t uuid;
	esp_gatt_perm_t perm;
	esp_gatt_char_prop_t property;

	uint16_t value_max_len;
	uint16_t value_len;
	/**
	 * The initial value. A copy of this value is created during
	 * initialization. May be null.
	 */
	uint8_t *value;

	uint16_t descr_count;
	/**
	 * List of descriptors to add to this characteristic. May be null.
	 */
	ble_desc_definition_t *descriptors;
} ble_chr_definition_t;

typedef void (*service_handles_cb_t)(
	uint16_t count, const uint16_t handles[count]
);

/**
 * Set the device name to use for the ble service.
 * Must be called before starting the ble service.
 *
 * @param name The device name to use. May not be more than 30 characters long
 * (excluding the terminating null byte).
 * @return Returns CUSTOM_BLE_OK if successfull, otherwise
 *     - CUSTOM_BLE_ALREADY_STARTED: The ble service has already been started
 *     using custom_ble_start.
 *     - CUSTOM_BLE_NAME_TOO_LONG: The provided name was too long.
 */
custom_ble_result_t custom_ble_set_name(const char *name);

// TODO: write docs
// Must be called after starting ble service.
// If this fails, you're kinda screwed, since the internal attribute count is
// Still incremented, with no way to decrement it from the outside. Yeah... ._.
// Blocks until handles_cb has been called with the resulting handles.
/**
 * Add a service with the specified list of characteristics and descriptors.
 *
 * The created handles are returned to a callback function. This function will
 * always be called before this function returns. In other words this function
 * blocks until the service has been created.
 *
 * Note: You should only call this function from a single thread.
 *
 *
 * @param service_uuid The uuid of the created service.
 * @param chr_count The length of chr.
 * @param chr A list of characteristic definitions, that specifies the
 * characteristics and their descriptors that should be added. All sub pointers
 * in this struct only need to live for the duration of this function call.
 * @param handles_cb This function will be called with a list of the created
 * service, characteristic, and descriptor handles. The service handle is always
 * the first in this list, followed by the characteristic and descriptor
 * handles. These appear in the order that they were given in the characteristic
 * list, with each characteristic handle appearing before the handles of its
 * descriptors.
 * @return TODO
 */
custom_ble_result_t custom_ble_add_service(
	esp_bt_uuid_t service_uuid, uint16_t chr_count,
	const ble_chr_definition_t chr[chr_count], service_handles_cb_t handles_cb
);

/**
 * Remove a service created with custom_ble_add_service.
 *
 * The specified service needs to be the last added service. This means that you
 * need to remove several services in the reverse order that how they were
 * added.
 *
 * Note: You should only call this function from a single thread.
 *
 * @param service_handle The service to remove. This should be a service handle
 * acquired through the handles callback function given to
 * custom_ble_add_service.
 * @return
 * - CUSTOM_BLE_OK:               The operation was successfull.
 * - CUSTOM_BLE_INVALID_HANDLE:   The given handle did not represent any
 *       existing service.
 * - CUSTOM_BLE_SERVICE_NOT_LAST: The given service was not most recently added
 *       service in the list of currently active services.
 * - CUSTOM_BLE_NOT_STARTED:      The ble service has not been started yet.
 * - CUSTOM_BLE_ESP_ERROR:        An error was generated when trying to remove
 *       the service using the ESP BLE APIs for an unknown reason. The server
 * might be left in an invalid state as a result of this.
 * - CUSTOM_BLE_INTERNAL_ERROR:   The internal list of attributes was out of
 *       order for an unknown reason. This shouldn't ever happen and the server
 *       is most likely in an invalid state.
 */
custom_ble_result_t custom_ble_remove_service(uint16_t service_handle);

/**
 * Get the current value of a characteristic or descriptor.
 *
 * Note: unsure if this works with descriptors...
 *
 *
 * @param attr_handle The characteristic or descriptor handle. This should be a
 * handle acquired through the handles callback function given to
 * custom_ble_add_service.
 * @param length Will be set to the length in bytes of the current value.
 * @param value Will be set to a pointer to the current value. TODO: Unsure how
 * long this pointer will live...
 * @return
 * - CUSTOM_BLE_OK:             The operation was successfull.
 * - CUSTOM_BLE_INVALID_HANDLE: The given characteristic or descriptor did not
 *      exist.
 * - CUSTOM_BLE_ESP_ERROR:      Some error was generated for an unknown reason
 *      by a call to the underlying ESP APIs.
 */
custom_ble_result_t custom_ble_get_attr_value(
	uint16_t attr_handle, uint16_t *length, const uint8_t **value
);

/**
 * Set the value of a characteristic or descriptor.
 *
 *
 * Calling this function automatically sends notifications and/or
 * indications if required.
 *
 * Note: unsure if this works with descriptors...
 * Note: You should only call this function from a single thread.
 *
 * @param attr_handle The characteristic or descriptor handle. This should be a
 * handle acquired through the handles callback function given to
 * custom_ble_add_service.
 * @param length The length of the value.
 * @param value The value that will be set as the current value. Does not need
 * to live longer than this function call (I think).
 * @return Sorry, this one won't tell you if the handle wasn't valid... ._.
 * - CUSTOM_BLE_OK:             The operation was successfull.
 * - CUSTOM_BLE_ESP_ERROR:      Some error was generated for an unknown reason
 *      by a call to the underlying ESP APIs.
 */
custom_ble_result_t custom_ble_set_attr_value(
	uint16_t attr_handle, uint16_t length, const uint8_t value[length]
);

/**
 * Start the ble server.
 *
 * Note: custom_ble_init has to have been called at some point before this.
 * Note: You should only call this function from a single thread.
 *
 * @return
 * - CUSTOM_BLE_OK: on success
 * - CUSTOM_BLE_INIT_FAILED: This is returned if custom_ble_init failed. This is
 *   typically due to memory allocation failing.
 */
custom_ble_result_t custom_ble_start();
void custom_ble_init();

#endif /* MAIN_BLE_CUSTOM_BLE_H_ */
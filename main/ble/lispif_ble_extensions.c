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

#include "lispif_ble_extensions.h"

#include <string.h>

#include "esp_bt_defs.h"

#include "custom_ble.h"
#include "utils.h"
#include "heap.h"
#include "lbm_defines.h"
#include "lbm_memory.h"
#include "extensions.h"

#include "commands.h"

/**
 * Add symbol to the symbol table if it doesn't already exist.
 */
static bool lbm_add_symbol_const_if_new(char *name, lbm_uint *id) {
	if (!lbm_get_symbol_by_name(name, id) && !lbm_add_symbol_const(name, id)) {
		return false;
	}

	return true;
}

/**
 * Reverse the elements of an array.
 *
 * @param dest Pointer where the reversed array will be placed. The destination
 * may overlap with the source array (so dest may be equal to src).
 * @param src The array to reverse.
 * @param len How many elements are in the array.
 * @param item_size The size in bytes of each element.
 */
static void array_reverse(
	void *dest, const void *src, size_t len, size_t item_size
) {

	memmove(dest, src, len * item_size);

	// source: https://stackoverflow.com/a/47745217/15507414
	void *temp[item_size];
	for (size_t i = 0; i < len / 2; i++) {
		memcpy(temp, dest + i * item_size, item_size);

		memcpy(
			dest + i * item_size, dest + (len - 1 - i) * item_size, item_size
		);
		memcpy(dest + (len - 1 - i) * item_size, temp, item_size);
	}
}

static lbm_uint symbol_uuid          = 0;
static lbm_uint symbol_prop          = 0;
static lbm_uint symbol_max_len       = 0;
static lbm_uint symbol_default_value = 0;
static lbm_uint symbol_descr         = 0;

static lbm_uint symbol_prop_read     = 0;
static lbm_uint symbol_prop_write    = 0;
static lbm_uint symbol_prop_write_nr = 0;
static lbm_uint symbol_prop_indicate = 0;
static lbm_uint symbol_prop_notify   = 0;

static bool register_symbols(void) {
	bool res = true;

	res = res && lbm_add_symbol_const_if_new("uuid", &symbol_uuid);
	res = res && lbm_add_symbol_const_if_new("prop", &symbol_prop);
	res = res && lbm_add_symbol_const_if_new("max-len", &symbol_max_len);
	res = res
		&& lbm_add_symbol_const_if_new("default-value", &symbol_default_value);
	res = res && lbm_add_symbol_const_if_new("descr", &symbol_descr);

	res = res && lbm_add_symbol_const_if_new("prop-read", &symbol_prop_read);
	res = res && lbm_add_symbol_const_if_new("prop-write", &symbol_prop_write);
	res = res
		&& lbm_add_symbol_const_if_new("prop-write-nr", &symbol_prop_write_nr);
	res = res
		&& lbm_add_symbol_const_if_new("prop-indicate", &symbol_prop_indicate);
	res = res
		&& lbm_add_symbol_const_if_new("prop-notify", &symbol_prop_notify);

	return res;
}

/**
 * Convert a lbm byte array into a esp uuid struct.
 * The bytes in the array should be big endian. In other words, the bytes of the
 * uuid should be placed in the same order as when you write out a uuid in text.
 */
static bool lbm_dec_uuid(lbm_value value, esp_bt_uuid_t *result) {
	if (!lbm_is_array_r(value)) {
		return false;
	}

	uint8_t *data = lbm_heap_array_get_data(value);

	switch (lbm_heap_array_get_size(value)) {
		case ESP_UUID_LEN_16: {
			uint16_t uuid = (uint16_t)data[1] + ((uint16_t)data[0] << 8);
			*result       = (esp_bt_uuid_t){
					  .len  = ESP_UUID_LEN_16,
					  .uuid = {.uuid16 = uuid},
            };

			return true;
		}
		case ESP_UUID_LEN_32: {
			uint32_t uuid = ((uint32_t)data[0] << 24)
				+ ((uint32_t)data[1] << 16) + ((uint32_t)data[2] << 8)
				+ (uint32_t)data[3];
			*result = (esp_bt_uuid_t){
				.len  = ESP_UUID_LEN_32,
				.uuid = {.uuid32 = uuid},
			};

			return true;
		}
		case ESP_UUID_LEN_128: {
			*result = (esp_bt_uuid_t){
				.len = ESP_UUID_LEN_128,
			};
			array_reverse(
				&result->uuid.uuid128, data, ESP_UUID_LEN_128, sizeof(uint8_t)
			);

			return true;
		}
	}

	return false;
}

static bool lbm_dec_ble_prop_flags(
	lbm_value value, esp_gatt_char_prop_t *dest
) {
	if (!lbm_is_list(value)) {
		return false;
	}

	*dest = 0;

	lbm_value next = value;
	while (lbm_is_cons(next)) {
		lbm_value this = lbm_car(next);
		next           = lbm_cdr(next);

		if (!lbm_is_symbol(this)) {
			return false;
		}

		lbm_uint sym = lbm_dec_sym(this);

		// TODO
		if (sym == symbol_prop_read) {
			*dest |= ESP_GATT_CHAR_PROP_BIT_READ;
		} else if (sym == symbol_prop_write_nr) {
			*dest |= ESP_GATT_CHAR_PROP_BIT_WRITE_NR;
		} else if (sym == symbol_prop_write) {
			*dest |= ESP_GATT_CHAR_PROP_BIT_WRITE;
		} else if (sym == symbol_prop_notify) {
			*dest |= ESP_GATT_CHAR_PROP_BIT_NOTIFY;
		} else if (sym == symbol_prop_indicate) {
			*dest |= ESP_GATT_CHAR_PROP_BIT_INDICATE;
		} else {
			// invalid property flag
			return false;
		}
	}

	return true;
}

typedef enum {
	PARSE_LBM_OK                  = 0,
	PARSE_LBM_INCORRECT_STRUCTURE = 1,
	PARSE_LBM_INVALID_TYPE        = 2,
	PARSE_LBM_TOO_MANY_ATTRIBUTES = 3,
	PARSE_LBM_MEMORY_ERROR        = 4,
} parse_lbm_result_t;

typedef struct {
	uint16_t start;
	uint16_t end;
} index_span_t;

typedef struct {
	uint16_t handle;
	uint8_t *value;
} attr_instance_t;

// A reference of this is passed to the ESP APIs when the user hasn't provided
// any byte array as a default value. It's fine that it's shared, since the api
// just copies the value either way.
static uint8_t default_zero = 0;

static parse_lbm_result_t parse_lbm_descr_def(
	lbm_value descr_def, ble_desc_definition_t *dest, uint16_t *used_attr_index
) {
	(void)used_attr_index;
	// This function shares a lot of code with parse_lbm_chr_def. Maybe they
	// can be merged somehow?

	if (!lbm_is_list(descr_def)) {
		return PARSE_LBM_INVALID_TYPE;
	}

	bool has_uuid          = false;
	bool has_max_len       = false;
	bool has_default_value = false;

	esp_bt_uuid_t uuid;
	uint16_t max_len;
	uint16_t value_len;
	uint8_t *default_value;

	lbm_value next = descr_def;
	while (lbm_is_cons(next)) {
		lbm_value this = lbm_car(next);
		next           = lbm_cdr(next);

		if (!lbm_is_cons(this) || !lbm_is_symbol(lbm_car(this))) {
			continue;
		}

		lbm_uint key    = lbm_dec_sym(lbm_car(this));
		lbm_value value = lbm_cdr(this);
		if (key == symbol_uuid) {
			has_uuid = true;
			if (!lbm_dec_uuid(value, &uuid)) {
				return PARSE_LBM_INVALID_TYPE;
			}
		} else if (key == symbol_max_len) {
			has_max_len = true;

			if (!lbm_is_number(value)) {
				return PARSE_LBM_INVALID_TYPE;
			}

			max_len = (uint16_t)lbm_dec_as_u32(value);
			if (max_len == 0) {
				return PARSE_LBM_INCORRECT_STRUCTURE;
			}
		} else if (key == symbol_default_value) {
			has_default_value = true;

			if (!lbm_is_array_r(value)) {
				return PARSE_LBM_INVALID_TYPE;
			}

			value_len = lbm_heap_array_get_size(value);
			if (value_len == 0) {
				// This might not even be necessary?
				// This is kindof a bad error to return here...
				return PARSE_LBM_INCORRECT_STRUCTURE;
			}

			default_value = lbm_heap_array_get_data(value);
		}
	}

	if (!has_uuid || !has_max_len) {
		return PARSE_LBM_INCORRECT_STRUCTURE;
	}
	if (!has_default_value) {
		value_len     = 1;
		default_value = &default_zero;
	}

	dest->uuid          = uuid;
	dest->perm          = ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE;
	dest->value_max_len = max_len;
	dest->value_len     = value_len;
	dest->value         = default_value;

	return PARSE_LBM_OK;
}

/**
 * Parse a lbm associative array into a characteristic definition. The structure
 * of said lbm value is checked.
 *
 * NOTE: Memory is alloced for dest->descriptors, that needs to be freed by the
 * caller using lbm_free.
 *
 * @return If the result wasn't successfull, you don't need to free the
 * descriptor list in dest->descriptors.
 * - PARSE_LBM_OK: The operation was successfull.
 * - PARSE_LBM_INCORRECT_STRUCTURE: chr_def had an invalid structure.
 * - PARSE_LBM_TOO_MANY_ATTRIBUTES: The total amount of created
 *   characteristics or descriptors has exceeded the configured amount.
 * - PARSE_LBM_MEMORY_ERROR: Allocating memory with malloc failed.
 */
static parse_lbm_result_t parse_lbm_chr_def(
	lbm_value chr_def, ble_chr_definition_t *dest,
	index_span_t *used_attr_indices
) {
	(void)used_attr_indices;

	if (!lbm_is_list(chr_def)) {
		return PARSE_LBM_INVALID_TYPE;
	}

	bool has_uuid          = false;
	bool has_prop          = false;
	bool has_max_len       = false;
	bool has_default_value = false;
	bool has_descr         = false;

	esp_bt_uuid_t uuid;
	esp_gatt_char_prop_t prop = 0;
	uint16_t max_len;
	uint16_t value_len;
	uint8_t *default_value;

	lbm_value descr_raw = ENC_SYM_NIL; // to get the compiler to shut up

	lbm_value next = chr_def;
	while (lbm_is_cons(next)) {
		lbm_value this = lbm_car(next);
		next           = lbm_cdr(next);

		if (!lbm_is_cons(this) || !lbm_is_symbol(lbm_car(this))) {
			continue;
		}

		lbm_uint key    = lbm_dec_sym(lbm_car(this));
		lbm_value value = lbm_cdr(this);
		if (key == symbol_uuid) {
			has_uuid = true;
			if (!lbm_dec_uuid(value, &uuid)) {
				return PARSE_LBM_INVALID_TYPE;
			}

		} else if (key == symbol_prop) {
			has_prop = true;
			if (!lbm_dec_ble_prop_flags(value, &prop)) {
				return PARSE_LBM_INVALID_TYPE;
			}
		} else if (key == symbol_max_len) {
			has_max_len = true;

			if (!lbm_is_number(value)) {
				return PARSE_LBM_INVALID_TYPE;
			}

			max_len = lbm_dec_as_u32(value);
			if (max_len == 0) {
				return PARSE_LBM_INCORRECT_STRUCTURE;
			}
		} else if (key == symbol_default_value) {
			has_default_value = true;

			if (!lbm_is_array_r(value)) {
				return PARSE_LBM_INVALID_TYPE;
			}

			value_len = lbm_heap_array_get_size(value);
			if (value_len == 0) {
				// This might not even be necessary?
				// This is kindof a bad error to return here...
				return PARSE_LBM_INCORRECT_STRUCTURE;
			}

			default_value = lbm_heap_array_get_data(value);
		} else if (key == symbol_descr) {
			has_descr = true;

			descr_raw = value;
		}
	}

	if (!has_uuid || !has_prop || !has_max_len) {
		return PARSE_LBM_INCORRECT_STRUCTURE;
	}
	if (!has_default_value) {
		value_len     = 1;
		default_value = &default_zero;
	}

	uint16_t descr_count = 0;
	if (has_descr) {
		if (!lbm_is_list(descr_raw)) {
			// lbm_free(chr_value);
			return PARSE_LBM_INCORRECT_STRUCTURE;
		}

		descr_count = lbm_list_length(descr_raw);
	}

	ble_desc_definition_t *descriptors =
		lbm_malloc_reserve(MAX(descr_count * sizeof(ble_desc_definition_t), 1));
	if (descriptors == NULL) {
		// lbm_free(chr_value);
		// next_attr_index = used_attr_indices->start;
		return PARSE_LBM_MEMORY_ERROR;
	}

	if (descr_count != 0) {
		lbm_value next = descr_raw;

		uint16_t i = 0;
		while (lbm_is_cons(next) && i < descr_count) {
			parse_lbm_result_t result =
				parse_lbm_descr_def(lbm_car(next), &descriptors[i], NULL);
			if (result != PARSE_LBM_OK) {
				stored_printf("descr parse lbm error: %d", result);

				lbm_free(descriptors);
				return result;
			}

			next = lbm_cdr(next);
			i++;
		}
	}

	dest->uuid     = uuid;
	dest->perm     = ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE;
	dest->property = prop;

	dest->value_max_len = max_len;
	dest->value_len     = value_len;
	dest->value         = default_value;

	dest->descr_count = descr_count;
	dest->descriptors = descriptors;

	return PARSE_LBM_OK;
}

static lbm_value prepared_handles_list;
static void store_handle_list(uint16_t count, const uint16_t handles[count]) {
	prepared_handles_list = ENC_SYM_NIL;
	for (int i = count - 1; i >= 0; i--) {
		prepared_handles_list =
			lbm_cons(lbm_enc_u(handles[i]), prepared_handles_list);
		if (prepared_handles_list == ENC_SYM_MERROR) {
			// TODO: deregister service.
			stored_printf("oh nose, memory error! BLE state is now invalid! :("
			);
			break;
		}
	}
}

/**
 * Parses chr_def into a compatible form and passes it on to custom_ble.c.
 * 
 * @return A list of the registered service, characteristic, and descriptor
 * handles on success. Otherwise either an eval_error or type_error symbol is
 * returned.
 */
static lbm_value add_service(esp_bt_uuid_t service_uuid, lbm_value chr_def) {
	if (!lbm_is_list(chr_def)) {
		return ENC_SYM_TERROR;
	}

	lbm_uint chr_count = lbm_list_length(chr_def);
	ble_chr_definition_t characteristics[chr_count];

	parse_lbm_result_t res_error;

	lbm_value next = chr_def;
	for (size_t i = 0; i < chr_count; i++) {
		if (!lbm_is_cons(next)) {
			for (size_t j = 0; j < i; j++) {
				lbm_free(characteristics[j].descriptors);
			}
			stored_printf("encountered not cons value");
			res_error = PARSE_LBM_INCORRECT_STRUCTURE;
			goto error;
		}
		index_span_t span;
		parse_lbm_result_t result =
			parse_lbm_chr_def(lbm_car(next), &characteristics[i], &span);

		if (result != PARSE_LBM_OK) {
			for (size_t j = 0; j < i; j++) {
				lbm_free(characteristics[j].descriptors);
			}
			stored_printf("chr parse lbm error: %d", result);
			res_error = result;
			goto error;
		}

		next = lbm_cdr(next);
	}

	stored_printf(
		"create custom ble service with %u characteristics", chr_count
	);
	custom_ble_result_t result = custom_ble_add_service(
		service_uuid, chr_count, characteristics, store_handle_list
	);
	for (size_t i = 0; i < chr_count; i++) {
		lbm_free(characteristics[i].descriptors);
	}

	if (result != CUSTOM_BLE_OK) {
		stored_printf("custom_ble_add_service error: %d", result);
		// this is very ugly semantics-wise...
		res_error = PARSE_LBM_INCORRECT_STRUCTURE;

		goto error;
	}

	if (prepared_handles_list == ENC_SYM_MERROR) {
		res_error = PARSE_LBM_MEMORY_ERROR;
		goto error;
	}

	return prepared_handles_list;

error:

	stored_printf("parse lbm error: %d", res_error);

	switch (res_error) {
		case PARSE_LBM_INVALID_TYPE: {
			return ENC_SYM_TERROR;
		}
		case PARSE_LBM_MEMORY_ERROR: {
			return ENC_SYM_MERROR;
		}
		case PARSE_LBM_INCORRECT_STRUCTURE:
		case PARSE_LBM_TOO_MANY_ATTRIBUTES:
		default: {
			return ENC_SYM_EERROR;
		}
	}
}

/**
 * signature (ble-init-app) -> bool
 *
 * @return Returns true the first time it's called, and nil every time after
 * that. If the internal init function fails, this function throws an
 * eval_error.
 */
static lbm_value ext_ble_init_app(lbm_value *args, lbm_uint argn) {
	(void)args;
	(void)argn;

	custom_ble_result_t result = custom_ble_start();
	switch (result) {
		case CUSTOM_BLE_OK: {
			return ENC_SYM_TRUE;
		}
		case CUSTOM_BLE_ALREADY_STARTED: {
			return ENC_SYM_NIL;
		}
		default: {
			return ENC_SYM_EERROR;
		}
	}
}

/**
 * signature: (ble-set-name name:byte-array) -> bool
 *
 * @return True if ble-init-app hasn't been called before, false if it has, or
 * eval_error if an error occurs, such as the name being longer than
 * CUSTOM_BLE_MAX_NAME_LEN.
 */
static lbm_value ext_ble_set_name(lbm_value *args, lbm_uint argn) {
	if (argn != 1 || !lbm_is_array_r(args[0])) {
		return ENC_SYM_TERROR;
	}

	const char *str = (char *)lbm_heap_array_get_data(args[0]);

	custom_ble_result_t result = custom_ble_set_name(str);
	switch (result) {
		case CUSTOM_BLE_OK: {
			return ENC_SYM_TRUE;
		}
		case CUSTOM_BLE_ALREADY_STARTED: {
			return ENC_SYM_NIL;
		}
		default: {
			return ENC_SYM_EERROR;
		}
	}
}

/**
 * signature: (ble-add-service service-uuid chrs)
 *
 * needs to be called after ble-init-app
 *
 * characteristic list example:
 * 	chrs = (list ...(
 * 		('uuid . uuid)
 * 		('prop . prop-value)
 *      ('max-len . number)
 *      [('default-value . byte-array)]
 * 		[('descr . (list ...(
 * 			('uuid . uuid)
 *          ('max-len . number)
 *          [('default-value . byte-array)]
 * 		)))]
 * 	))
 * where
 * 	uuid = byte-array of length 2, 4, or 16
 * 	prop-value = ([prop-read] [prop-write] [prop-write-nr] [prop-indicate]
 * 		[prop-notify])
 *
 */
static lbm_value ext_ble_add_service(lbm_value *args, lbm_uint argn) {
	if (argn != 2) {
		return ENC_SYM_TERROR;
	}

	esp_bt_uuid_t uuid;
	if (!lbm_dec_uuid(args[0], &uuid)) {
		return ENC_SYM_TERROR;
	}

	return add_service(uuid, args[1]);
}

/**
 * signature: (ble-remove-service service-handle:number)
 */
static lbm_value ext_ble_remove_service(lbm_value *args, lbm_uint argn) {
	if (argn != 1 && !lbm_is_number(args[0])) {
		return ENC_SYM_TERROR;
	}

	uint16_t handle = (uint16_t)lbm_dec_as_u32(args[0]);

	custom_ble_result_t result = custom_ble_remove_service(handle);
	if (result != CUSTOM_BLE_OK) {
		stored_printf("custom_ble_remove_service failed, result: %d", result);
		return ENC_SYM_EERROR;
	}

	return ENC_SYM_TRUE;
}

/**
 * signature: (ble-attr-get-value handle: number) -> byte-array
 */
static lbm_value ext_ble_attr_get_value(lbm_value *args, lbm_uint argn) {
	if (argn != 1 && !lbm_is_number(args[0])) {
		return ENC_SYM_TERROR;
	}

	uint16_t handle = (uint16_t)lbm_dec_as_u32(args[0]);

	uint16_t len;
	const uint8_t *value;

	custom_ble_result_t result =
		custom_ble_get_attr_value(handle, &len, &value);
	if (result != CUSTOM_BLE_OK) {
		return ENC_SYM_EERROR;
	}

	uint8_t *result_data = lbm_malloc_reserve(len);
	if (result_data == NULL) {
		return ENC_SYM_MERROR;
	}

	memcpy(result_data, value, len);

	lbm_value ret;
	if (!lbm_lift_array(&ret, (char *)result_data, len)) {
		return ENC_SYM_MERROR;
	}

	return ret;
}

/**
 * signature: (ble-attr-set-value handle:number value:byte-array)
 */
static lbm_value ext_ble_attr_set_value(lbm_value *args, lbm_uint argn) {
	if (argn != 2 && !lbm_is_number(args[0]) && !lbm_is_array_r(args[1])) {
		return ENC_SYM_TERROR;
	}

	uint16_t handle = (uint16_t)lbm_dec_as_u32(args[0]);
	uint16_t len    = (uint16_t)lbm_heap_array_get_size(args[1]);
	uint8_t *value  = lbm_heap_array_get_data(args[1]);
	if (value == NULL) {
		// Maybe return internal error here?
		return ENC_SYM_EERROR;
	}

	custom_ble_result_t result = custom_ble_set_attr_value(handle, len, value);
	if (result != CUSTOM_BLE_OK) {
		return ENC_SYM_EERROR;
	}

	return ENC_SYM_TRUE;
}

/**
 * signature: (ble-get-services) -> handles
 * where
 *   handles = list of numbers
 *
 * Gets all currently active service handles. They are returned in the order
 * they were created, so that the last service is the one you're allowed to
 * remove.
 */
static lbm_value ext_ble_get_services(lbm_value *args, lbm_uint argn) {
	uint16_t count = custom_ble_service_count();

	uint16_t handles[count];

	custom_ble_get_services(count, handles);

	// array_reverse(handles, handles, count, sizeof(uint16_t));

	lbm_value list = ENC_SYM_NIL;
	for (uint16_t i = 0; i < count; i++) {
		list = lbm_cons(lbm_enc_u(handles[i]), list);
		if (list == ENC_SYM_MERROR) {
			return list;
		}
	}
	return list;
}

/**
 * signature: (ble-get-attrs service_handle:number) -> handles
 * where
 *   handles = list of numbers
 *
 * Gets all characteristic or descriptor handles that belong to the given
 * service. An eval_error is thrown if the handle isn't valid.
 */
static lbm_value ext_ble_get_attrs(lbm_value *args, lbm_uint argn) {
	if (argn != 1 && !lbm_is_number(args[0])) {
		return ENC_SYM_TERROR;
	}

	uint16_t service_handle = (uint16_t)lbm_dec_as_u32(args[0]);

	int16_t count = custom_ble_attr_count(service_handle);
	if (count < 0) {
		return ENC_SYM_EERROR;
	}

	uint16_t handles[count];

	if (custom_ble_get_attrs(service_handle, count, handles) < 0) {
		// should never happen
		return ENC_SYM_EERROR;
	}

	lbm_value list = ENC_SYM_NIL;
	for (uint16_t i = 0; i < count; i++) {
		list = lbm_cons(lbm_enc_u(handles[i]), list);
		if (list == ENC_SYM_MERROR) {
			return list;
		}
	}
	return list;
}

void lispif_load_ble_extensions(void) {
	register_symbols();

	lbm_add_extension("ble-init-app", ext_ble_init_app);
	lbm_add_extension("ble-set-name", ext_ble_set_name);
	lbm_add_extension("ble-add-service", ext_ble_add_service);
	lbm_add_extension("ble-remove-service", ext_ble_remove_service);
	lbm_add_extension("ble-attr-get-value", ext_ble_attr_get_value);
	lbm_add_extension("ble-attr-set-value", ext_ble_attr_set_value);
	lbm_add_extension("ble-get-services", ext_ble_get_services);
	lbm_add_extension("ble-get-attrs", ext_ble_get_attrs);
}
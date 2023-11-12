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

#include <stdbool.h>

#include "lbm_types.h"
#include "lbm_defines.h"
#include "heap.h"
#include "lbm_flat_value.h"


/**
 * Add symbol to the symbol table if it doesn't already exist.
 */
bool lbm_add_symbol_const_if_new(char *name, lbm_uint *id);

static inline bool lbm_is_bool(lbm_value value) {
	return (lbm_is_symbol(value) && (lbm_dec_sym(value) == SYM_NIL || lbm_dec_sym(value) == SYM_TRUE));
}

// nvm, this function already existed in the standard lbm library...
// /**
//  * Access the data of an lbm byte array as a string.
//  * 
//  * @param value The value whose content will be returned if it's a byte array.
//  * The array may be a read-only array! (Contrary to lbm_heap_array_get_data
//  * which only takes writable arrays.)
//  * @return The str data, or NULL if value was not a byte array.
// */
// char *lbm_dec_str(lbm_value value);

/**
 * Dec lbm value as a boolean.
 * 
 * @param value The lbm value to interpret as a boolean.
 * @return False if the value was nil or true otherwise.
*/
static inline bool lbm_dec_bool(lbm_value value) {
	return !lbm_is_symbol_nil(value);
}

/**
 * Extract the array header struct from a lbm value array.
 * 
 * The type of the value is checked to be at least be a readable array (doesn't
 * have to be writeable though).
 * 
 * @param value The lbm value to convert to an array header struct.
 * @return The extracted pointer to the array header struct if value was a
 * readable array. Null is returned otherwise. (@note: Null is also returned if
 * the lbm_value contained a null pointer, @todo: unsure exactly what layer that would
 * be right now though...)
*/
lbm_array_header_t *lbm_dec_array_header(lbm_value value);

/**
 * Extract array data from a lbm value.
 * 
 * The type of the value is checked to be at least be a readable array (doesn't
 * have to be writeable though).
 * 
 * @param value The lbm value to potentially extract array data from.
 * @return The extracted pointer if value was a readable array. Null is returned
 * otherwise. (@note: Null is also returned if the lbm_value contained a null
 * pointer, @todo: unsure exactly what layer that would be right now though...)
*/
void *lbm_dec_array_data(lbm_value value);

/**
 * Create an lbm list by allocating a chain of con cells, with the value nil as
 * elements. This list can then be filled with actual values later if the caller
 * so chooses.
 * 
 * @param len How many elements long the allocated list should be.
 * @return The allocated list, or ENC_SYM_MERROR when out of memory.
*/
lbm_value lbm_allocate_empty_list(lbm_uint len);

/**
 * Create a 2-dimensional lbm list filled with the value nil, which can then be
 * filled with actual values later if the caller so chooses.
 * 
 * @example If you were to allocate a grid of height 4 and width 2, the
 * following list would be returned.
 * ```
 * (
 *     (nil nil)
 *     (nil nil)
 *     (nil nil)
 *     (nil nil)
 * )
 * ```
 * 
 * @param height The length of the single outer list.
 * @param width The length of each of the many sublists contained in the out
 * list.
 * @return The allocated list, or ENC_SYM_MERROR when out of memory.
*/
lbm_value lbm_allocate_empty_list_grid(lbm_uint height, lbm_uint width);

/**
 * Shrink an lbm array to the new specified size.
 * 
 * @param lbm_value The array to shrink. Should hold an LBM value that
 * corresponds to an array.
 * @param new_size The new smaller array size.
 * @return Bool indicating if the array was successfully shrunk. False is
 * returned if array didn't hold a byte array value, or if new_len was larger
 * than the original size.
*/
bool lbm_array_shrink(lbm_value array, lbm_uint new_size);

/**
 * Check if the number of arguments is the specified range. Sets error-reason if
 * result is false.
 * 
 * The range specified is inclusive!
 * 
 * @param argn Number of arguments.
 * @param n_min Minimum number of arguments.
 * @param n_max Maximum number of arguments.
 * 
*/
bool lbm_check_argn_range(lbm_uint argn, lbm_uint n_min, lbm_uint n_max);

#define LBM_CHECK_ARGN_RANGE(min, max) if (!lbm_check_argn_range(argn, (min), (max))) {return ENC_SYM_EERROR;}

/**
 * Construct a flat value containing a single lbm array from a c array,
 * automatically allocating a flat value of the required size.
 * 
 * Don't forget that it's the callers responsibility to free the buf member of
 * the returned flat value struct using lbm_free (unless you pass it to
 * lbm_unblock_ctx which takes ownership of the value for instance).
 * 
 * @param result A caller-allocated flat value struct that will be filled with the
 * result on success. Is usually just allocated on the stack. The returned
 * struct will be ready to be used, with lbm_finish_flatten already beeing
 * called.
 * @param data Pointer to the array to pack.
 * @param size Length of the data in bytes.
 * @return If necessary allocations were successfull. 
*/
bool f_pack_array(lbm_flat_value_t *result, void * data, size_t size);
/*
	Copyright 2022 Benjamin Vedder	benjamin@vedder.se

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

/*
 * Native library C interface for the VESC Express.
 *
 * This is a NEW interface, separate from the bldc (STM32 motor controller)
 * vesc_c_if. It starts from the platform-neutral core - LispBM access,
 * threads, timing, mutexes/semaphores, memory and printf - and contains no
 * motor, CAN or peripheral functions. More slots will be appended over time.
 *
 * Compatibility rules:
 *  - New function pointers are ONLY ever added at the END of the struct, so
 *    libs built against older headers keep working on newer firmware. Added
 *    slots are null-pointers on firmware that predates them - a lib that
 *    wants to run on older firmware can check them for NULL.
 *  - VESC_C_IF_VERSION is bumped ONLY on a breaking layout change. INIT_START
 *    compares it against the firmware's table (first slot, read before any
 *    function in the table is called) and fails the lib init on mismatch.
 */

#ifndef VESC_C_IF_H
#define VESC_C_IF_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

// Layout version of the vesc_c_if struct below. Only bumped on breaking
// changes; appended slots do NOT bump it.
#define VESC_C_IF_VERSION 1

#define NATIVE_LIB_MAGIC 0xCAFEBABE

// Container magic for relocatable libs that the firmware copies into RAM
// and patches at load time. Used on targets that cannot run position-
// independent code in place (ESP32-S3 / Xtensa). Layout: magic,
// version, code_size, data_size, entry_offset, reloc_count, relocs[],
// code[], data[].
#define NATIVE_LIB_RELOC_MAGIC 0xCAFEBABF

// System tick, as returned by system_time_ticks
typedef uint32_t systime_t;

#ifdef IS_VESC_LIB
// LBM types, provided by lispbm.h when compiling the firmware itself.
typedef uint32_t lbm_value;
typedef uint32_t lbm_type;
typedef uint32_t lbm_cid;

typedef uint32_t lbm_uint;
typedef int32_t  lbm_int;
typedef float    lbm_float;

typedef struct {
	uint8_t *buf;
	lbm_uint buf_size;
	lbm_uint buf_pos;
} lbm_flat_value_t;

typedef struct {
	lbm_uint size;            /// Number of elements
	lbm_uint *data;           /// pointer to lbm_memory array or C array.
} lbm_array_header_t;

typedef lbm_value (*extension_fptr)(lbm_value*,lbm_uint);

// For double precision literals
#define D(x) 				((double)x##L)
#endif

typedef bool (*load_extension_fptr)(char*,extension_fptr);

typedef void* lib_thread;
typedef void* lib_mutex;
typedef void* lib_semaphore;

/*
 * Function pointer struct. Always add new function pointers to the end in
 * order to not break compatibility with old binaries. If a function is not
 * available (e.g. in an old firmware) it will be a null-pointer.
 */
typedef struct {
	// Interface layout version, always the first slot. Read by INIT_START
	// before anything else in the table is used.
	uint32_t if_version;

	// LBM: extensions, symbols and errors
	load_extension_fptr lbm_add_extension;
	int (*lbm_set_error_reason)(char *str);
	int (*lbm_add_symbol_const)(const char *, lbm_uint *);
	int (*lbm_get_symbol_by_name)(const char *name, lbm_uint* id);

	// LBM: evaluator control and messaging
	void (*lbm_block_ctx_from_extension)(void);
	bool (*lbm_unblock_ctx)(lbm_cid, lbm_flat_value_t*);
	bool (*lbm_unblock_ctx_unboxed)(lbm_cid cid, lbm_value unboxed);
	lbm_cid (*lbm_get_current_cid)(void);
	int (*lbm_send_message)(lbm_cid cid, lbm_value msg);
	void (*lbm_pause_eval_with_gc)(uint32_t num_free);
	void (*lbm_continue_eval)(void);
	bool (*lbm_eval_is_paused)(void);

	// LBM: heap and values
	lbm_value (*lbm_cons)(lbm_value car, lbm_value cdr);
	lbm_value (*lbm_car)(lbm_value val);
	lbm_value (*lbm_cdr)(lbm_value val);
	lbm_value (*lbm_list_destructive_reverse)(lbm_value list);
	bool (*lbm_create_byte_array)(lbm_value *value, lbm_uint num_elt);

	// LBM: encoding
	lbm_value (*lbm_enc_i)(lbm_int x);
	lbm_value (*lbm_enc_u)(lbm_uint x);
	lbm_value (*lbm_enc_char)(uint8_t x);
	lbm_value (*lbm_enc_float)(float f);
	lbm_value (*lbm_enc_u32)(uint32_t u);
	lbm_value (*lbm_enc_i32)(int32_t i);
	lbm_value (*lbm_enc_sym)(lbm_uint s);

	// LBM: decoding
	float (*lbm_dec_as_float)(lbm_value val);
	uint32_t (*lbm_dec_as_u32)(lbm_value val);
	int32_t (*lbm_dec_as_i32)(lbm_value val);
	uint8_t (*lbm_dec_char)(lbm_value x);
	char* (*lbm_dec_str)(lbm_value);
	lbm_uint (*lbm_dec_sym)(lbm_value x);

	// LBM: type checks
	bool (*lbm_is_byte_array)(lbm_value val);
	bool (*lbm_is_cons)(lbm_value x);
	bool (*lbm_is_number)(lbm_value x);
	bool (*lbm_is_char)(lbm_value x);
	bool (*lbm_is_symbol)(lbm_value x);
	bool (*lbm_is_symbol_nil)(lbm_uint);
	bool (*lbm_is_symbol_true)(lbm_uint);

	// LBM: symbol constants
	lbm_uint lbm_enc_sym_nil;
	lbm_uint lbm_enc_sym_true;
	lbm_uint lbm_enc_sym_terror;
	lbm_uint lbm_enc_sym_eerror;
	lbm_uint lbm_enc_sym_merror;

	// LBM: flat values
	bool (*lbm_start_flatten)(lbm_flat_value_t *v, size_t buffer_size);
	bool (*lbm_finish_flatten)(lbm_flat_value_t *v);
	bool (*f_cons)(lbm_flat_value_t *v);
	bool (*f_sym)(lbm_flat_value_t *v, lbm_uint sym);
	bool (*f_i)(lbm_flat_value_t *v, lbm_int i);
	bool (*f_b)(lbm_flat_value_t *v, uint8_t b);
	bool (*f_i32)(lbm_flat_value_t *v, int32_t w);
	bool (*f_u32)(lbm_flat_value_t *v, uint32_t w);
	bool (*f_float)(lbm_flat_value_t *v, float f);
	bool (*f_i64)(lbm_flat_value_t *v, int64_t w);
	bool (*f_u64)(lbm_flat_value_t *v, uint64_t w);
	bool (*f_lbm_array)(lbm_flat_value_t *v, uint32_t num_elts, uint8_t *data);

	// Os: time and sleep
	void (*sleep_ms)(uint32_t ms);
	void (*sleep_us)(uint32_t us);
	float (*system_time)(void); // Time since boot in seconds
	float (*ts_to_age_s)(systime_t ts); // Age of timestamp in seconds
	// Time since boot in system ticks (see SYSTEM_TICK_RATE_HZ). Use
	// ts_to_age_s to get the age of a timestamp in seconds; it handles
	// overflows.
	systime_t (*system_time_ticks)(void);
	void (*sleep_ticks)(systime_t ticks);
	// High resolution timer for short busy-wait sleeps and time
	// measurement, in microseconds.
	uint32_t (*timer_time_now)(void);
	float (*timer_seconds_elapsed_since)(uint32_t time);
	void (*timer_sleep)(float seconds);

	// Os: memory and IO
	int (*printf)(const char *str, ...);
	void* (*malloc)(size_t bytes);
	void (*free)(void *ptr);

	// Os: threads
	lib_thread (*spawn)(void (*fun)(void *arg), size_t stack_size, const char *name, void *arg);
	void (*request_terminate)(lib_thread thd);
	bool (*should_terminate)(void);
	// Set priority of current thread.
	// Range: -5 to 5, -5 is lowest, 0 is normal, 5 is highest
	void (*thread_set_priority)(int priority);
	void** (*get_arg)(uint32_t prog_addr);

	// Os: mutex
	lib_mutex (*mutex_create)(void); // Use VESC_IF->free on the mutex when done with it
	void (*mutex_lock)(lib_mutex);
	void (*mutex_unlock)(lib_mutex);

	// Os: semaphore
	lib_semaphore (*sem_create)(void); // Use VESC_IF->free on the semaphore when done with it
	void (*sem_wait)(lib_semaphore);
	void (*sem_signal)(lib_semaphore);
	bool (*sem_wait_to)(lib_semaphore, systime_t); // Returns false on timeout
	void (*sem_reset)(lib_semaphore);
} vesc_c_if;

typedef struct {
	void (*stop_fun)(void *arg);
	void *arg;
	uint32_t base_addr;
} lib_info;

// System tick rate. Can be used to convert system ticks to time
#define SYSTEM_TICK_RATE_HZ 1000

/*
 * Address of the firmware-side C interface table. It must match the address
 * of the .libif section in main/linker_libif_<target>.ld for the target the
 * firmware (and the native library) is built for.
 *
 * The firmware build picks the target up from sdkconfig automatically. When
 * building a native library out of tree, define the CONFIG_IDF_TARGET_*
 * macro matching the hardware you are building for, e.g.
 * -DCONFIG_IDF_TARGET_ESP32C3=1. A library only works on the target it was
 * built for.
 */
#if defined(__has_include)
#if __has_include("sdkconfig.h")
#include "sdkconfig.h"
#endif
#endif

#if defined(CONFIG_IDF_TARGET_ESP32C3)
#define VESC_IF		((vesc_c_if*)(0x3FCDBE00))
#elif defined(CONFIG_IDF_TARGET_ESP32S3)
#define VESC_IF		((vesc_c_if*)(0x3FCE8800))
#elif defined(CONFIG_IDF_TARGET_ESP32C6)
#define VESC_IF		((vesc_c_if*)(0x4087B800))
#elif defined(CONFIG_IDF_TARGET_ESP32P4)
#define VESC_IF		((vesc_c_if*)(0x4FF3A000))
#else
#error "Unknown ESP target. Define CONFIG_IDF_TARGET_ESP32C3, -S3, -C6 or -P4 when building a native library."
#endif

// Put this at the beginning of your source file
#define HEADER		volatile int __attribute__((__section__(".program_ptr"))) prog_ptr;

// Init function
#define INIT_FUN	bool __attribute__((__section__(".init_fun"))) init

// Put this at the start of the init function. The version check reads the
// first table slot only - safe even when the firmware carries a different
// interface layout.
#define INIT_START	(void)prog_ptr; \
					if (VESC_IF->if_version != VESC_C_IF_VERSION) { \
						return false; \
					}

// Address of this program in memory
#define PROG_ADDR	((uint32_t)&prog_ptr)

// The argument that was set in the init function (same as the one you get in stop_fun)
#define ARG			(*VESC_IF->get_arg(PROG_ADDR))

extern volatile int prog_ptr;

#endif  // VESC_C_IF_H

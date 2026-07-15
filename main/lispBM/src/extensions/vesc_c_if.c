/*
    Copyright 2022‑2024 Benjamin Vedder     <benjamin@vedder.se>
    Copyright 2022      Joel Svensson      <svenssonjoel@yahoo.se>

    This file is part of the VESC firmware and is released under the
    terms of the GNU General Public License, version 3 (or any later).

    -----------------------------------------------------------------
    FreeRTOS / ESP‑IDF PORT
    -----------------------------------------------------------------
    All ChibiOS threading primitives have been replaced with their
    FreeRTOS counterparts so that `(spawn …)` and related Lisp helpers
    work unmodified on ESP32 targets.

    This implements the VESC Express native lib interface (see
    extensions/vesc_c_if.h) - a fresh interface separate from the bldc
    one, starting from the platform-neutral core: LispBM access,
    threads, timing, mutexes/semaphores, memory and printf. New slots
    are appended to the struct; VESC_C_IF_VERSION is only bumped on
    breaking layout changes.
*/

#pragma GCC optimize("Os")
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "esp_heap_caps.h"
#include "esp_memory_utils.h"
#include "heap_memory_layout.h"
#include "commands.h"
#include "extensions.h"
#include "lbm_flat_value.h"
#include "lispif.h"
#include "lispbm.h"
#include "utils.h"
#include "extensions/vesc_c_if.h"
#include "freertos/task.h"

_Static_assert(sizeof(vesc_c_if) <= 2048, "cif pad too small");

typedef struct {
	const char *name;
	void *arg;
	void (*func)(void*);
	volatile bool should_terminate;
	TaskHandle_t handle;
	UBaseType_t base_prio;
} lib_thd_info;

#define LIB_MAX_THREADS 20
static lib_thd_info *lib_thread_infos[LIB_MAX_THREADS] = {0};
static size_t lib_thread_infos_cnt = 0;

// Optional: protect lib_thread_infos[] edits/reads if accessed from multiple tasks
static portMUX_TYPE lib_thr_mux = portMUX_INITIALIZER_UNLOCKED;
#define LIB_THR_LOCK()   portENTER_CRITICAL(&lib_thr_mux)
#define LIB_THR_UNLOCK() portEXIT_CRITICAL(&lib_thr_mux)

#define LIB_NUM_MAX 10

static lib_info loaded_libs[LIB_NUM_MAX] = {0};

// The flash (IROM) address of each loaded lib's container, i.e. the value
// the lisp code passes to load-native-lib / unload-native-lib. For XIP libs
// this equals base_addr; for RAM-loaded (relocated) libs it differs.
static uint32_t lib_flash_addr[LIB_NUM_MAX] = {0};

// Heap allocation backing a RAM-loaded lib, NULL for XIP libs.
static void *lib_ram_alloc[LIB_NUM_MAX] = {0};

// Second allocation backing a RAM-loaded lib's data region (S3), NULL
// otherwise.
static void *lib_ram_data[LIB_NUM_MAX] = {0};

__attribute__((section(".libif"))) static volatile union {
	vesc_c_if cif;
	char pad[2048];
} cif;

// The .libif section is placed at a fixed, target-specific address by
// main/linker_libif_<target>.ld so that native libs can find the interface
// table through the VESC_IF macro. Keep the heap allocator away from it.
SOC_RESERVE_MEMORY_REGION((intptr_t)&cif, (intptr_t)&cif + sizeof(cif), vesc_libif);

static bool lib_init_done = false;

static bool lib_is_func_valid(void *func) {
	return esp_ptr_executable(func);
}

static void lib_sleep_ms(uint32_t ms) {
	vTaskDelay(pdMS_TO_TICKS(ms));
}

static void lib_sleep_us(uint32_t us) {
	if (us >= 1000) {
		vTaskDelay(pdMS_TO_TICKS(us / 1000));
		us %= 1000;
	}
	if (us)
		esp_rom_delay_us(us);
}

static float lib_system_time(void) {
	return UTILS_AGE_S(0);
}

static float lib_ts_to_age_s(TickType_t ts) {
	return UTILS_AGE_S(ts);
}

static void lib_thd(void *arg) {
	lib_thd_info *t = (lib_thd_info*)arg;

	// Set thread-local storage for should_terminate check
	vTaskSetThreadLocalStoragePointer(NULL, 0, t);

	t->func(t->arg);

	// Task finished, remove from global tracking
	for (size_t i = 0; i < lib_thread_infos_cnt; i++) {
		if (lib_thread_infos[i] == t) {
			// Shift down remaining entries
			for (size_t j = i; j < lib_thread_infos_cnt - 1; j++) {
				lib_thread_infos[j] = lib_thread_infos[j + 1];
			}
			lib_thread_infos[--lib_thread_infos_cnt] = NULL;
			break;
		}
	}

	lbm_free(t);
	vTaskDelete(NULL);  // clean self-termination
}


static bool lib_should_terminate(void) {
	lib_thd_info *info = (lib_thd_info*) pvTaskGetThreadLocalStoragePointer(NULL, 0);
	return info && info->should_terminate;
}
_Static_assert(
	configNUM_THREAD_LOCAL_STORAGE_POINTERS > 0,
	"Need ≥1 TLS pointer for lib thread bookkeeping"
);
lib_thread lispif_spawn(void (*func)(void*), size_t stack_size, const char *name, void *arg) {
	if (!lib_is_func_valid(func)) {
		commands_printf_lisp("Invalid function address. Must be static.");
		return 0;
	}

	if (lib_thread_infos_cnt >= LIB_MAX_THREADS) {
		commands_printf_lisp("Thread limit reached.");
		return 0;
	}

	lib_thd_info *info = lbm_malloc_reserve(sizeof(lib_thd_info));
	if (!info) {
		commands_printf_lisp("Failed to allocate thread info");
		return 0;
	}

	info->arg = arg;
	info->func = func;
	info->name = name;
	info->should_terminate = false;

	TaskHandle_t thd = NULL;
	BaseType_t result = xTaskCreatePinnedToCore(
		lib_thd,
		name ? name : "lib-thd",
		stack_size,
		info,
		tskIDLE_PRIORITY + 5,
		&thd,
		tskNO_AFFINITY
	);

	if (result == pdPASS && thd != NULL) {
		info->handle = thd;
		info->base_prio = uxTaskPriorityGet(thd);
		lib_thread_infos[lib_thread_infos_cnt++] = info;
		return (lib_thread)thd;
	} else {
		commands_printf_lisp("Thread creation failed");
		lbm_free(info);
		return 0;
	}
}


static void lib_request_terminate(lib_thread thd) {
	TaskHandle_t handle = (TaskHandle_t)thd;

	for (size_t i = 0; i < lib_thread_infos_cnt; i++) {
		if (lib_thread_infos[i]->handle == handle) {
			lib_thread_infos[i]->should_terminate = true;

			// Wait for task to self-terminate
			int timeout = 2000;
			while (eTaskGetState(handle) != eDeleted && timeout-- > 0) {
				vTaskDelay(pdMS_TO_TICKS(1));
			}

			if (timeout <= 0) {
				commands_printf_lisp("Thread did not exit. Crashing...");
				vTaskDelay(pdMS_TO_TICKS(20));
				abort();
			}

			return;
		}
	}

	commands_printf_lisp("Thread handle not found");
}


static inline UBaseType_t clamp_prio(int p) {
	// ESP-IDF sets FREERTOS_MAX_PRIORITIES via sdkconfig; typical is 25.
	const UBaseType_t maxp = configMAX_PRIORITIES - 1;
	const UBaseType_t minp = tskIDLE_PRIORITY + 1;
	if (p < (int)minp)
		return minp;
	if (p > (int)maxp)
		return maxp;
	return (UBaseType_t)p;
}

static void lib_thread_set_priority(int delta /* -5..5 */) {
	// Find our bookkeeping record for the CURRENT task
	lib_thd_info *info =
		(lib_thd_info *)pvTaskGetThreadLocalStoragePointer(NULL, 0);
	if (!info || info->handle != xTaskGetCurrentTaskHandle()) {
		lbm_set_error_reason(
			"thread_set_priority must be called from a lib thread"
		);
		return;
	}

	// Normalize the requested delta into absolute target priority
	// 0 => baseline, +1 => one level higher than baseline, etc.
	int target       = (int)info->base_prio + delta;
	UBaseType_t newp = clamp_prio(target);

	vTaskPrioritySet(info->handle, newp);
}

static void **lib_get_arg(uint32_t prog_addr) {
	uint32_t p = (uint32_t)utils_drom_to_irom((void *)prog_addr);

	for (int i = 0; i < LIB_NUM_MAX; i++) {
		uint32_t base = loaded_libs[i].base_addr;
		if (!base)
			continue;

		if (p == base + 4u) {
			return &loaded_libs[i].arg;
		}
	}
	return NULL;
}

static bool lib_create_byte_array(lbm_value *value, lbm_uint num_elt) {
	return lbm_heap_allocate_array(value, num_elt);
}

static bool lib_eval_is_paused(void) {
	return lbm_get_eval_state() == EVAL_CPS_STATE_PAUSED;
}

static lib_mutex lib_mutex_create(void) {
	SemaphoreHandle_t *m = lbm_malloc_reserve(sizeof(SemaphoreHandle_t));
	if (!m)
		return NULL;
	*m = xSemaphoreCreateMutex();
	if (!*m) {
		lbm_free(m);
		return NULL;
	}
	return (lib_mutex)m;
}

static void lib_mutex_lock(lib_mutex m) {
	xSemaphoreTake(*((SemaphoreHandle_t *)m), portMAX_DELAY);
}

static void lib_mutex_unlock(lib_mutex m) {
	xSemaphoreGive(*((SemaphoreHandle_t *)m));
}

static lib_semaphore lib_sem_create(void) {
	SemaphoreHandle_t *s = lbm_malloc_reserve(sizeof(SemaphoreHandle_t));
	if (!s)
		return NULL;
	*s = xSemaphoreCreateCounting(0xFFFF, 0);
	if (!*s) {
		lbm_free(s);
		return NULL;
	}
	return (lib_semaphore)s;
}

static void lib_sem_wait(lib_semaphore s) {
	xSemaphoreTake(*((SemaphoreHandle_t *)s), portMAX_DELAY);
}

static void lib_sem_signal(lib_semaphore s) {
	xSemaphoreGive(*((SemaphoreHandle_t *)s));
}

static bool lib_sem_wait_to(lib_semaphore s, TickType_t timeout_ticks) {
	return xSemaphoreTake(*((SemaphoreHandle_t *)s), timeout_ticks) == pdPASS;
}

static void lib_sem_reset(lib_semaphore s) {
	SemaphoreHandle_t h = *((SemaphoreHandle_t *)s);
	while (xSemaphoreTake(h, 0) == pdPASS) { /* drain */
	}
}

static bool lib_add_extension(char *sym_str, extension_fptr ext) {
	if (sym_str[0] != 'e' || sym_str[1] != 'x' || sym_str[2] != 't'
		|| sym_str[3] != '-') {
		lbm_set_error_reason("Error: Extensions must start with ext-");
		return false;
	}

	return lbm_add_extension(sym_str, ext);
}

static int lib_lbm_set_error_reason(char *str) {
	lbm_set_error_reason(str);
	return 1;
}

// High resolution timer for short busy-wait sleeps and time measurement
uint32_t lib_timer_time_now() {
	return (uint32_t)(esp_timer_get_time()); // microseconds
}

float lib_timer_seconds_elapsed_since(uint32_t time_us) {
	uint32_t now_us = lib_timer_time_now();
	return (now_us - time_us) / 1000000.0f;
}

void lib_timer_sleep(float seconds) {
	if (seconds <= 0)
		return;
	uint32_t us = (uint32_t)(seconds * 1000000.0f);
	while (us >= 2000) {
		vTaskDelay(pdMS_TO_TICKS(1));
		us -= 1000;
	}
	if (us)
		esp_rom_delay_us(us);
}

void lispif_stop_lib(void) {
	// 1) Call stop_fun for all loaded libs (mirrors STM32)
	for (int i = 0; i < LIB_NUM_MAX; i++) {
		if (loaded_libs[i].stop_fun) {
			if (lib_is_func_valid(loaded_libs[i].stop_fun)) {
				loaded_libs[i].stop_fun(loaded_libs[i].arg);
			}
			loaded_libs[i].stop_fun  = NULL;
			loaded_libs[i].base_addr = 0;
			loaded_libs[i].arg       = NULL;
			lib_flash_addr[i]        = 0;
		}
	}
	// 2) Terminate remaining lib threads. Snapshot the handles under the
	// lock, but request termination outside of it as lib_request_terminate
	// blocks and blocking is not allowed in a critical section.
	TaskHandle_t handles[LIB_MAX_THREADS];
	size_t handle_cnt = 0;

	LIB_THR_LOCK();
	for (size_t i = 0; i < lib_thread_infos_cnt; i++) {
		if (lib_thread_infos[i] && lib_thread_infos[i]->handle) {
			handles[handle_cnt++] = lib_thread_infos[i]->handle;
		}
	}
	LIB_THR_UNLOCK();

	for (size_t i = 0; i < handle_cnt; i++) {
		lib_request_terminate(handles[i]);
	}

	// 3) Free RAM-loaded lib images. Done last, after every lib thread is
	// gone, as their code lives in these allocations.
	for (int i = 0; i < LIB_NUM_MAX; i++) {
		if (lib_ram_alloc[i]) {
			heap_caps_free(lib_ram_alloc[i]);
			lib_ram_alloc[i] = NULL;
		}
		if (lib_ram_data[i]) {
			heap_caps_free(lib_ram_data[i]);
			lib_ram_data[i] = NULL;
		}
	}
}

lbm_value ext_load_native_lib(lbm_value *args, lbm_uint argn) {
	lbm_value res = lbm_enc_sym(SYM_EERROR);

	// Expect a single numeric argument containing the IROM base address
	if (argn != 1 || !lbm_is_number(args[0])) {
		return res;
	}

	// The linker script and VESC_IF must agree on where the interface table
	// lives, otherwise libs would read garbage function pointers.
	if ((uintptr_t)&cif != (uintptr_t)VESC_IF) {
		lbm_set_error_reason("Native lib interface address mismatch (firmware bug)");
		return res;
	}

	if (!lib_init_done) {
		// Zero the padding beyond the struct; slots appended to the
		// interface after this firmware was built read as NULL.
		memset((char *)cif.pad, 0, 2048);

		cif.cif.if_version = VESC_C_IF_VERSION;

		// LBM
		cif.cif.lbm_add_extension            = lib_add_extension;
		cif.cif.lbm_block_ctx_from_extension = lbm_block_ctx_from_extension;
		cif.cif.lbm_unblock_ctx              = lbm_unblock_ctx;
		cif.cif.lbm_get_current_cid          = lbm_get_current_cid;
		cif.cif.lbm_set_error_reason         = lib_lbm_set_error_reason;
		cif.cif.lbm_pause_eval_with_gc       = lbm_pause_eval_with_gc;
		cif.cif.lbm_continue_eval            = lbm_continue_eval;
		cif.cif.lbm_send_message             = lbm_send_message;
		cif.cif.lbm_eval_is_paused           = lib_eval_is_paused;

		cif.cif.lbm_cons                     = lbm_cons;
		cif.cif.lbm_car                      = lbm_car;
		cif.cif.lbm_cdr                      = lbm_cdr;
		cif.cif.lbm_list_destructive_reverse = lbm_list_destructive_reverse;
		cif.cif.lbm_create_byte_array        = lib_create_byte_array;

		cif.cif.lbm_add_symbol_const   = lbm_add_symbol_const;
		cif.cif.lbm_get_symbol_by_name = lbm_get_symbol_by_name;

		cif.cif.lbm_enc_i     = lbm_enc_i;
		cif.cif.lbm_enc_u     = lbm_enc_u;
		cif.cif.lbm_enc_char  = lbm_enc_char;
		cif.cif.lbm_enc_float = lbm_enc_float;
		cif.cif.lbm_enc_u32   = lbm_enc_u32;
		cif.cif.lbm_enc_i32   = lbm_enc_i32;
		cif.cif.lbm_enc_sym   = lbm_enc_sym;

		cif.cif.lbm_dec_as_float = lbm_dec_as_float;
		cif.cif.lbm_dec_as_u32   = lbm_dec_as_u32;
		cif.cif.lbm_dec_as_i32   = lbm_dec_as_i32;
		cif.cif.lbm_dec_char     = lbm_dec_char;
		cif.cif.lbm_dec_str      = lbm_dec_str;
		cif.cif.lbm_dec_sym      = lbm_dec_sym;

		cif.cif.lbm_is_byte_array = lbm_is_array_r;
		cif.cif.lbm_is_cons       = lbm_is_cons;
		cif.cif.lbm_is_number     = lbm_is_number;
		cif.cif.lbm_is_char       = lbm_is_char;
		cif.cif.lbm_is_symbol     = lbm_is_symbol;

		cif.cif.lbm_enc_sym_nil    = ENC_SYM_NIL;
		cif.cif.lbm_enc_sym_true   = ENC_SYM_TRUE;
		cif.cif.lbm_enc_sym_terror = ENC_SYM_TERROR;
		cif.cif.lbm_enc_sym_eerror = ENC_SYM_EERROR;
		cif.cif.lbm_enc_sym_merror = ENC_SYM_MERROR;

		cif.cif.lbm_is_symbol_nil  = lbm_is_symbol_nil;
		cif.cif.lbm_is_symbol_true = lbm_is_symbol_true;

		// Os
		cif.cif.sleep_ms          = lib_sleep_ms;
		cif.cif.sleep_us          = lib_sleep_us;
		cif.cif.system_time       = lib_system_time;
		cif.cif.ts_to_age_s       = lib_ts_to_age_s;
		cif.cif.printf            = commands_printf_lisp;
		cif.cif.malloc            = lbm_malloc_reserve;
		cif.cif.free              = lbm_free;
		cif.cif.spawn             = lispif_spawn;
		cif.cif.request_terminate = lib_request_terminate;
		cif.cif.should_terminate  = lib_should_terminate;
		cif.cif.get_arg           = lib_get_arg;

		// Mutex
		cif.cif.mutex_create = lib_mutex_create;
		cif.cif.mutex_lock   = lib_mutex_lock;
		cif.cif.mutex_unlock = lib_mutex_unlock;

		// High resolution timer for short busy-wait sleeps and time measurement
		cif.cif.timer_time_now              = lib_timer_time_now;
		cif.cif.timer_seconds_elapsed_since = lib_timer_seconds_elapsed_since;
		cif.cif.timer_sleep                 = lib_timer_sleep;

		// Flat values
		cif.cif.lbm_start_flatten  = lbm_start_flatten;
		cif.cif.lbm_finish_flatten = lbm_finish_flatten;
		cif.cif.f_b                = f_b;
		cif.cif.f_cons             = f_cons;
		cif.cif.f_float            = f_float;
		cif.cif.f_i                = f_i;
		cif.cif.f_i32              = f_i32;
		cif.cif.f_i64              = f_i64;
		cif.cif.f_lbm_array        = f_lbm_array;
		cif.cif.f_sym              = f_sym;
		cif.cif.f_u32              = f_u32;
		cif.cif.f_u64              = f_u64;

		// Unblock unboxed
		cif.cif.lbm_unblock_ctx_unboxed = lbm_unblock_ctx_unboxed;

		// System time
		cif.cif.system_time_ticks = xTaskGetTickCount;
		cif.cif.sleep_ticks       = vTaskDelay;

		// Semaphores
		cif.cif.sem_create  = lib_sem_create;
		cif.cif.sem_wait    = lib_sem_wait;
		cif.cif.sem_signal  = lib_sem_signal;
		cif.cif.sem_wait_to = lib_sem_wait_to;
		cif.cif.sem_reset   = lib_sem_reset;

		cif.cif.thread_set_priority = lib_thread_set_priority;

		lib_init_done = true;
	}

	// Read IROM header base directly
	uint32_t irom_base = lbm_dec_as_u32(args[0]);

	// Basic pointer/alignment sanity
	if (irom_base == 0 || (irom_base & 0x3) != 0) {
		lbm_set_error_reason("Invalid IROM base pointer");
		return res;
	}

	// Validate native header magic. Read through the DROM alias, as data
	// reads through the instruction bus fault on some targets.
	const uint8_t *container_drom = utils_irom_to_drom((void *)irom_base);
	uint32_t magic_be = 0;
	memcpy(&magic_be, container_drom, sizeof(magic_be));

	bool is_reloc = magic_be == __builtin_bswap32(NATIVE_LIB_RELOC_MAGIC);
	if (!is_reloc && magic_be != __builtin_bswap32(NATIVE_LIB_MAGIC)) {
		lbm_set_error_reason("Magic number not found at IROM address");
		return res;
	}

	// Duplicate check by container flash address
	for (int i = 0; i < LIB_NUM_MAX; i++) {
		if (loaded_libs[i].stop_fun != NULL
			&& lib_flash_addr[i] == irom_base) {
			lbm_set_error_reason("Library already loaded");
			return res;
		}
	}

	int slot = -1;
	for (int i = 0; i < LIB_NUM_MAX; i++) {
		if (loaded_libs[i].stop_fun == NULL) {
			slot = i;
			break;
		}
	}
	if (slot < 0) {
		lbm_set_error_reason("Library table full");
		return res;
	}

	// base_addr is where the lib image lives at runtime (prog_ptr at +4),
	// entry_addr is the init function.
	uint32_t base_addr;
	uint32_t entry_addr;
	void *ram_alloc = NULL;
	void *ram_data = NULL;

	if (is_reloc) {
#if CONFIG_IDF_TARGET_ESP32S3 && CONFIG_ESP_SYSTEM_MEMPROT_FEATURE
		// The exec-heap allocation below can never succeed with memory
		// protection enabled, so fail with a message that says exactly
		// what is wrong with this firmware build.
		lbm_set_error_reason("This firmware was built with "
			"CONFIG_ESP_SYSTEM_MEMPROT_FEATURE=y - native libs on the "
			"ESP32-S3 need a build with it disabled");
		return res;
#elif CONFIG_IDF_TARGET_ESP32S3
		// Xtensa cannot run position-independent code in place, so the
		// container carries region-relative relocations and the image is
		// copied to RAM in two parts: the code region goes to executable
		// memory (any exec block works - including pure-IRAM without a
		// data alias, since code is only word-accessed) and the data
		// region to any byte-accessible internal RAM (including the
		// DRAM-only spare). This keeps native libs out of the contested
		// D/IRAM that LispBM needs. Container layout after the magic:
		// version, code_size, data_size, entry_offset, reloc_count (all
		// LE u32), relocs[], code[], data[].
		uint32_t version, code_size, data_size, entry_offset, reloc_count;
		memcpy(&version, container_drom + 4, 4);
		memcpy(&code_size, container_drom + 8, 4);
		memcpy(&data_size, container_drom + 12, 4);
		memcpy(&entry_offset, container_drom + 16, 4);
		memcpy(&reloc_count, container_drom + 20, 4);

		if (version != 2) {
			lbm_set_error_reason("Native lib container version mismatch - "
				"rebuild the lib with the current vesc_pkg c_libs");
			return res;
		}

		if (code_size < 4 || code_size > 0x40000 || (code_size & 3)
			|| data_size < 8 || data_size > 0x40000 || (data_size & 3)
			|| entry_offset >= code_size || (entry_offset & 3)
			|| reloc_count > (code_size + data_size) / 4) {
			lbm_set_error_reason("Invalid native lib container");
			return res;
		}

		uint32_t *code_ram = heap_caps_malloc(
			code_size, MALLOC_CAP_EXEC | MALLOC_CAP_INTERNAL);
		uint8_t *data_ram = heap_caps_malloc(
			data_size, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
		if (!code_ram || !data_ram) {
			static char err_buf[96];
			snprintf(err_buf, sizeof(err_buf),
				"Out of memory for lib: code %u (largest %u), data %u (largest %u)",
				(unsigned)code_size,
				(unsigned)heap_caps_get_largest_free_block(
					MALLOC_CAP_EXEC | MALLOC_CAP_INTERNAL),
				(unsigned)data_size,
				(unsigned)heap_caps_get_largest_free_block(
					MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
			if (code_ram) heap_caps_free(code_ram);
			if (data_ram) heap_caps_free(data_ram);
			lbm_set_error_reason(err_buf);
			return res;
		}

		const uint8_t *relocs = container_drom + 24;
		const uint8_t *code_src = relocs + reloc_count * 4;
		const uint8_t *data_src = code_src + code_size;

		// The code block may be pure IRAM, which only allows aligned
		// 32-bit accesses - copy and patch it word-wise.
		for (uint32_t i = 0; i < code_size / 4; i++) {
			uint32_t w;
			memcpy(&w, code_src + i * 4, 4);
			code_ram[i] = w;
		}
		memcpy(data_ram, data_src, data_size);

		// Relocation entries: bit31 = target is code, bit30 = the word
		// sits in the data region, low bits = region-relative offset of
		// the word. Stored words are region-relative target offsets.
		bool patch_ok = true;
		for (uint32_t r = 0; r < reloc_count; r++) {
			uint32_t e;
			memcpy(&e, relocs + r * 4, 4);
			uint32_t off = e & 0x3FFFFFFF;
			uint32_t add = (e & 0x80000000)
				? (uint32_t)code_ram : (uint32_t)data_ram;

			if (e & 0x40000000) {
				if ((off & 3) || off + 4 > data_size) {
					patch_ok = false;
					break;
				}
				uint32_t word;
				memcpy(&word, data_ram + off, 4);
				word += add;
				memcpy(data_ram + off, &word, 4);
			} else {
				if ((off & 3) || off + 4 > code_size) {
					patch_ok = false;
					break;
				}
				code_ram[off / 4] += add;
			}
		}

		uint32_t inner_magic = 0;
		memcpy(&inner_magic, data_ram, 4);
		if (!patch_ok || inner_magic != __builtin_bswap32(NATIVE_LIB_MAGIC)) {
			heap_caps_free(code_ram);
			heap_caps_free(data_ram);
			lbm_set_error_reason("Invalid relocation table in native lib");
			return res;
		}

		// Make the copied and patched code visible to instruction fetch.
		__asm__ __volatile__("memw\n\tisync\n\t" ::: "memory");

		base_addr  = (uint32_t)data_ram;
		entry_addr = (uint32_t)code_ram + entry_offset;
		ram_alloc  = code_ram;
		ram_data   = data_ram;
#else
		lbm_set_error_reason(
			"Relocatable libs are only supported on the ESP32-S3");
		return res;
#endif
	} else {
		// XIP: runs in place from flash. Entry is after the header:
		// magic(4) + prog_addr(4) = 8 bytes.
		base_addr  = irom_base;
		entry_addr = irom_base + 8;
	}

	loaded_libs[slot].base_addr = base_addr;
	lib_flash_addr[slot]        = irom_base;
	lib_ram_alloc[slot]         = ram_alloc;
	lib_ram_data[slot]          = ram_data;

	bool ok = ((bool (*)(lib_info *))entry_addr)(&loaded_libs[slot]);

	if (ok && loaded_libs[slot].stop_fun != NULL) {
		void *stop_fun_irom = utils_drom_to_irom(loaded_libs[slot].stop_fun);
		if (lib_is_func_valid(stop_fun_irom)) {
			loaded_libs[slot].stop_fun = stop_fun_irom;
			return lbm_enc_sym(SYM_TRUE);
		}
		lbm_set_error_reason("Invalid stop function. Must be static.");
	} else if (ok) {
		lbm_set_error_reason("Library init failed - no stop function set");
	} else {
		lbm_set_error_reason("Library init failed");
	}

	// Rollback
	loaded_libs[slot].stop_fun  = NULL;
	loaded_libs[slot].base_addr = 0;
	loaded_libs[slot].arg       = NULL;
	lib_flash_addr[slot]        = 0;
	if (lib_ram_alloc[slot]) {
		heap_caps_free(lib_ram_alloc[slot]);
		lib_ram_alloc[slot] = NULL;
	}
	if (lib_ram_data[slot]) {
		heap_caps_free(lib_ram_data[slot]);
		lib_ram_data[slot] = NULL;
	}

	return res;
}

lbm_value ext_unload_native_lib(lbm_value *args, lbm_uint argn) {
	lbm_value res = lbm_enc_sym(SYM_EERROR);

	if (argn != 1 || !lbm_is_number(args[0])) {
		return res;
	}

	uint32_t irom_base = lbm_dec_as_u32(args[0]);

	for (int i = 0; i < LIB_NUM_MAX; i++) {
		if (loaded_libs[i].stop_fun != NULL
			&& lib_flash_addr[i] == irom_base) {
			// The stop function must stop everything the lib started,
			// including its threads, before returning.
			if (lib_is_func_valid(loaded_libs[i].stop_fun)) {
				loaded_libs[i].stop_fun(loaded_libs[i].arg);
			}
			loaded_libs[i].stop_fun  = NULL;
			loaded_libs[i].base_addr = 0;
			loaded_libs[i].arg       = NULL;
			lib_flash_addr[i]        = 0;
			if (lib_ram_alloc[i]) {
				heap_caps_free(lib_ram_alloc[i]);
				lib_ram_alloc[i] = NULL;
			}
			if (lib_ram_data[i]) {
				heap_caps_free(lib_ram_data[i]);
				lib_ram_data[i] = NULL;
			}

			return lbm_enc_sym(SYM_TRUE);
		}
	}

	lbm_set_error_reason("Library not loaded");
	return res;
}

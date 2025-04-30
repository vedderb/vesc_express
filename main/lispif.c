/*
	Copyright 2023 Benjamin Vedder	benjamin@vedder.se
	Copyright 2023 Joel Svensson    svenssonjoel@yahoo.se

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

#include <malloc.h>

#include "eval_cps.h"
#include "main.h"
#include "lispif.h"
#include "commands.h"
#include "terminal.h"
#include "buffer.h"
#include "lispbm.h"
#include "mempools.h"
#include "flash_helper.h"
#include "conf_general.h"
#include "lbm_prof.h"
#include "esp_timer.h"
#include "utils.h"

#define GC_STACK_SIZE			160
#define PRINT_STACK_SIZE		128
#ifndef EXTENSION_STORAGE_SIZE
#define EXTENSION_STORAGE_SIZE	320
#endif
#ifndef USER_EXTENSION_STORAGE_SIZE
#define USER_EXTENSION_STORAGE_SIZE 0
#endif
#define PROF_DATA_NUM			30
#define EXT_LOAD_CALLBACK_LEN	10

static size_t heap_size = 0;
static size_t mem_size = 0;
static size_t bitmap_size = 0;

static lbm_cons_t *heap;
static uint32_t *memory_array;
static uint32_t *bitmap_array;
static lbm_extension_t extension_storage[EXTENSION_STORAGE_SIZE + USER_EXTENSION_STORAGE_SIZE];

static lbm_const_heap_t const_heap;
static volatile lbm_uint *const_heap_ptr = 0;
static int const_heap_max_ind = 0;

static lbm_string_channel_state_t string_tok_state;
static lbm_char_channel_t string_tok;
static lbm_buffered_channel_state_t buffered_tok_state;
static lbm_char_channel_t buffered_string_tok;
static bool string_tok_valid = false;

static TaskHandle_t eval_task;
static volatile bool lisp_thd_running = false;
static SemaphoreHandle_t lbm_mutex;

static int repl_cid = -1;
static lbm_cid repl_cid_for_buffer = -1;
static char *repl_buffer = 0;
static volatile TickType_t repl_time = 0;
static int restart_cnt = 0;

static esp_timer_handle_t prof_timer;
static void prof_timer_callback(void* arg);
static lbm_prof_t prof_data[PROF_DATA_NUM];
static volatile bool prof_running = false;
const esp_timer_create_args_t periodic_timer_args = {
		.callback = &prof_timer_callback,
};

// Extension load callbacks
void(*ext_load_callbacks[EXT_LOAD_CALLBACK_LEN])(void) = {0};

/*
 * TODO:
 * - Do not restart LBM after wdt reboot?
 */

// Private functions
static uint32_t timestamp_callback(void);
static void sleep_callback(uint32_t us);
static bool const_heap_write(lbm_uint ix, lbm_uint w);
static void eval_thread(void *arg);

void lispif_init(void) {
	heap_size = (2048 + 512);
	mem_size = LBM_MEMORY_SIZE_32K;
	bitmap_size = LBM_MEMORY_BITMAP_SIZE_32K;

	if (backup.config.wifi_mode == WIFI_MODE_DISABLED &&
			backup.config.ble_mode == BLE_MODE_DISABLED) {
		heap_size *= 2;
		mem_size *= 3;
		bitmap_size *= 3;
	} else if (backup.config.wifi_mode == WIFI_MODE_DISABLED ||
			backup.config.ble_mode == BLE_MODE_DISABLED) {
		heap_size *= 2;
		mem_size *= 2;
		bitmap_size *= 2;
	}

	heap = memalign(8, heap_size * sizeof(lbm_cons_t));
	memory_array = heap_caps_malloc(mem_size * sizeof(uint32_t), MALLOC_CAP_DMA);
	bitmap_array = heap_caps_malloc(bitmap_size * sizeof(uint32_t), MALLOC_CAP_DMA);

	memset(&buffered_tok_state, 0, sizeof(buffered_tok_state));
	lbm_mutex = xSemaphoreCreateMutex();
	lispif_restart(false, true, true);
	lbm_set_eval_step_quota(50);
}

int lispif_get_restart_cnt(void) {
	return restart_cnt;
}

void lispif_lock_lbm(void) {
	xSemaphoreTake(lbm_mutex, portMAX_DELAY);
}

void lispif_unlock_lbm(void) {
	xSemaphoreGive(lbm_mutex);
}

static void print_ctx_info(eval_context_t *ctx, void *arg1, void *arg2) {
	(void) arg1;
	(void) arg2;

	char output[128];

	int print_ret = lbm_print_value(output, sizeof(output), ctx->r);

	commands_printf_lisp("--------------------------------");
	commands_printf_lisp("ContextID: %u", ctx->id);

	if (ctx->name) {
		commands_printf_lisp("Context Name: %s", ctx->name);
	}

	commands_printf_lisp("Stack SP: %u",  ctx->K.sp);
	commands_printf_lisp("Stack SP max: %u", lbm_get_max_stack(&ctx->K));
	commands_printf_lisp("Result%s: %s", print_ret ? "" : " (trunc)", output);
}

static void sym_it(const char *str) {
	bool sym_name_flash = lbm_symbol_in_flash((char *)str);
	bool sym_entry_flash = lbm_symbol_list_entry_in_flash((char *)str);
	commands_printf_lisp("[Name: %s, Entry: %s]: %s\n",
			sym_name_flash ? "FLASH" : "L_MEM",
					sym_entry_flash ? "FLASH" : "L_MEM",
							str);
}

static void prof_timer_callback(void* arg) {
	(void)arg;
	lbm_prof_sample();
}

static bool pause_eval(uint32_t num_free, uint32_t timeout_ms) {
	int timeout_cnt = timeout_ms;

	if (num_free > 0) {
		lbm_pause_eval_with_gc(num_free);
	} else {
		lbm_pause_eval();
	}

	while (lbm_get_eval_state() != EVAL_CPS_STATE_PAUSED && timeout_cnt > 0) {
		vTaskDelay(1 / portTICK_PERIOD_MS);
		timeout_cnt--;
	}

	return timeout_cnt > 0;
}

void lispif_process_cmd(unsigned char *data, unsigned int len,
		void(*reply_func)(unsigned char *data, unsigned int len)) {
	COMM_PACKET_ID packet_id;

	packet_id = data[0];
	data++;
	len--;

	switch (packet_id) {
	case COMM_LISP_SET_RUNNING: {
		bool ok = false;
		bool running = data[0];

		if (!running) {
			ok = pause_eval(0, 2000);
		} else {
			ok = lispif_restart(true, true, true);
		}

		int32_t ind = 0;
		uint8_t send_buffer[50];
		send_buffer[ind++] = packet_id;
		send_buffer[ind++] = ok;
		reply_func(send_buffer, ind);
	} break;


	case COMM_LISP_GET_STATS: {
		float cpu_use = 0.0;
		float heap_use = 0.0;
		float mem_use = 0.0;

		if (lisp_thd_running) {
			uint32_t timeTot = portGET_RUN_TIME_COUNTER_VALUE();
			portALT_GET_RUN_TIME_COUNTER_VALUE(timeTot);
			if (timeTot > 0) {
				TaskStatus_t stat;
				vTaskGetInfo(eval_task, &stat, pdFALSE, 0);
				static uint32_t time_last = 0;
				static uint32_t time_task_last = 0;
				cpu_use = 100.0 * (float)(stat.ulRunTimeCounter - time_task_last) / (float)(timeTot - time_last);
				time_last = timeTot;
				time_task_last = stat.ulRunTimeCounter;
			} else {
				cpu_use = 11.0;
			}
		} else {
			break;
		}

		bool print_all = true;
		if (len > 0) {
			print_all = data[0];
		}

		if (lbm_heap_state.gc_num > 0) {
			heap_use = 100.0 * (float)(heap_size - lbm_heap_state.gc_last_free) / (float)heap_size;
		}

		mem_use = 100.0 * (float)(lbm_memory_num_words() - lbm_memory_num_free()) / (float)lbm_memory_num_words();

		uint8_t *send_buffer_global = mempools_get_packet_buffer();
		int32_t ind = 0;

		send_buffer_global[ind++] = packet_id;
		buffer_append_float16(send_buffer_global, cpu_use, 1e2, &ind);
		buffer_append_float16(send_buffer_global, heap_use, 1e2, &ind);
		buffer_append_float16(send_buffer_global, mem_use, 1e2, &ind);

		// Stack. Currently unused
		buffer_append_float16(send_buffer_global, 0, 1e2, &ind);

		// Result. Currently unused.
		send_buffer_global[ind++] = '\0';

		if (pause_eval(0, 2000)) {
			lbm_value *glob_env = lbm_get_global_env();
			for (int i = 0; i < GLOBAL_ENV_ROOTS; i ++) {
				if (ind > 300) {
					break;
				}

				lbm_value curr = glob_env[i];
				while (lbm_type_of(curr) == LBM_TYPE_CONS) {
					lbm_value key_val = lbm_car(curr);
					if (lbm_type_of(lbm_car(key_val)) == LBM_TYPE_SYMBOL && lbm_is_number(lbm_cdr(key_val))) {
						const char *name = lbm_get_name_by_symbol(lbm_dec_sym(lbm_car(key_val)));

						if (print_all ||
								((name[0] == 'v' || name[0] == 'V') &&
										(name[1] == 't' || name[1] == 'T'))) {
							strcpy((char*)(send_buffer_global + ind), name);
							ind += strlen(name) + 1;
							buffer_append_float32_auto(send_buffer_global, lbm_dec_as_float(lbm_cdr(key_val)), &ind);
						}
					}

					if (ind > 300) {
						break;
					}

					curr = lbm_cdr(curr);
				}
			}
		}

		lbm_continue_eval();

		reply_func(send_buffer_global, ind);
		mempools_free_packet_buffer(send_buffer_global);
	} break;

	case COMM_LISP_REPL_CMD: {
		if (UTILS_AGE_S(repl_time) <= 0.5) {
			return;
		}

		if (!lisp_thd_running) {
			lispif_restart(true, false, true);
		}

		if (lisp_thd_running) {
			lispif_lock_lbm();
			char *str = (char*)data;

			if (len <= 1) {
				commands_printf_lisp(">");
			} else if (len >= 5 && strncmp(str, ":help", 5) == 0) {
				commands_printf_lisp("== Special Commands ==");
				commands_printf_lisp(
						":help\n"
						"  Print this help text");
				commands_printf_lisp(
						":info\n"
						"  Print info about memory usage, allocated arrays and garbage collection");
				commands_printf_lisp(
						":prof start\n"
						"  Start profiler");
				commands_printf_lisp(
						":prof stop\n"
						"  Stop profiler");
				commands_printf_lisp(
						":prof report\n"
						"  Print profiler report");
				commands_printf_lisp(
						":env\n"
						"  Print current environment and variables");
				commands_printf_lisp(
						":ctxs\n"
						"  Print context (threads) info");
				commands_printf_lisp(
						":symbols\n"
						"  Print symbol names");
				commands_printf_lisp(
						":reset\n"
						"  Reset LBM");
				commands_printf_lisp(
						":pause\n"
						"  Pause LBM");
				commands_printf_lisp(
						":continue\n"
						"  Continue running LBM");
				commands_printf_lisp(
						":undef <symbol_name>\n"
						"  Undefine symbol");
				commands_printf_lisp(
						":verb\n"
						"  Toggle verbose error messages");
				commands_printf_lisp(
						":state\n"
						"  Print evaluator state");
				commands_printf_lisp(" ");
				commands_printf_lisp("Anything else will be evaluated as an expression in LBM.");
				commands_printf_lisp(" ");
			} else if (len >= 5 && strncmp(str, ":info", 5) == 0) {
				commands_printf_lisp("--(LISP HEAP)--\n");
				commands_printf_lisp("Heap size: %u Bytes\n", heap_size * 8);
				commands_printf_lisp("Used cons cells: %d\n", heap_size - lbm_heap_num_free());
				commands_printf_lisp("Free cons cells: %d\n", lbm_heap_num_free());
				commands_printf_lisp("GC counter: %d\n", lbm_heap_state.gc_num);
				commands_printf_lisp("Recovered: %d\n", lbm_heap_state.gc_recovered);
				commands_printf_lisp("Recovered arrays: %u\n", lbm_heap_state.gc_recovered_arrays);
				commands_printf_lisp("Marked: %d\n", lbm_heap_state.gc_marked);
				commands_printf_lisp("GC SP max: %u (size %u)\n", lbm_get_max_stack(&lbm_heap_state.gc_stack), lbm_heap_state.gc_stack.size);
				commands_printf_lisp("--(Symbol and Array memory)--\n");
				commands_printf_lisp("Memory size: %u bytes\n", lbm_memory_num_words() * 4);
				commands_printf_lisp("Memory free: %u bytes\n", lbm_memory_num_free() * 4);
				commands_printf_lisp("Longest block free: %u bytes\n", lbm_memory_longest_free() * 4);
				commands_printf_lisp("Allocated arrays: %u\n", lbm_heap_state.num_alloc_arrays);
				commands_printf_lisp("Symbol table size: %u Bytes\n", lbm_get_symbol_table_size());
				commands_printf_lisp("Symbol table size flash: %u Bytes\n", lbm_get_symbol_table_size_flash());
				commands_printf_lisp("Symbol name size: %u Bytes\n", lbm_get_symbol_table_size_names());
				commands_printf_lisp("Symbol name size flash: %u Bytes\n", lbm_get_symbol_table_size_names_flash());
				commands_printf_lisp("Extensions: %u, max %u\n", lbm_get_num_extensions(), lbm_get_max_extensions());
				commands_printf_lisp("--(Flash)--\n");
				commands_printf_lisp("Size: %u Bytes\n", const_heap.size);
				commands_printf_lisp("Used cells: %d\n", const_heap.next);
				commands_printf_lisp("Free cells: %d\n", const_heap.size / 4 - const_heap.next);
				flast_stats stats = flash_helper_stats();
				commands_printf_lisp("Erase Cnt Tot: %d\n", stats.erase_cnt_tot);
				commands_printf_lisp("Erase Cnt Max Sector: %d\n", stats.erase_cnt_max);
				commands_printf_lisp("Num sectors erased: %d\n", stats.erased_sector_num);
			} else if (strncmp(str, ":prof start", 11) == 0) {
				if (prof_running) {
					lbm_prof_init(prof_data, PROF_DATA_NUM);
					commands_printf_lisp("Profiler restarted\n");
				} else {
					lbm_prof_init(prof_data, PROF_DATA_NUM);
					prof_running = true;
					esp_timer_create(&periodic_timer_args, &prof_timer);
					// Use a period that isn't a multiple if the eval thread periods
					esp_timer_start_periodic(prof_timer, 571);
					commands_printf_lisp("Profiler started\n");
				}
			} else if (strncmp(str, ":prof stop", 10) == 0) {
				if (prof_running) {
					prof_running = false;
					esp_timer_stop(prof_timer);
				}
				commands_printf_lisp("Profiler stopped. Issue command ':prof report' for statistics\n");
			} else if (strncmp(str, ":prof report", 12) == 0) {
				lbm_uint num_sleep = lbm_prof_get_num_sleep_samples();
				lbm_uint num_system = lbm_prof_get_num_system_samples();
				lbm_uint tot_samples = lbm_prof_get_num_samples();
				lbm_uint tot_gc = 0;
				commands_printf_lisp("CID\tName\tSamples\t%%Load\t%%GC");
				for (int i = 0; i < PROF_DATA_NUM; i ++) {
					if (prof_data[i].cid == -1) break;
					tot_gc += prof_data[i].gc_count;
					commands_printf_lisp("%d\t%s\t%u\t%.3f\t%.3f",
							prof_data[i].cid,
							prof_data[i].name,
							prof_data[i].count,
							(double)(100.0 * ((float)prof_data[i].count) / (float) tot_samples),
							(double)(100.0 * ((float)prof_data[i].gc_count) / (float)prof_data[i].count));
				}
				commands_printf_lisp(" ");
				commands_printf_lisp("GC:\t%u\t%f%%\n", tot_gc, (double)(100.0 * ((float)tot_gc / (float)tot_samples)));
				commands_printf_lisp("System:\t%u\t%f%%\n", num_system, (double)(100.0 * ((float)num_system / (float)tot_samples)));
				commands_printf_lisp("Sleep:\t%u\t%f%%\n", num_sleep, (double)(100.0 * ((float)num_sleep / (float)tot_samples)));
				commands_printf_lisp("Total:\t%u samples\n", tot_samples);
			} else if (strncmp(str, ":env", 4) == 0) {
				if (pause_eval(0, 1000)) {
					lbm_value *glob_env = lbm_get_global_env();
					char output[128];
					for (int i = 0; i < GLOBAL_ENV_ROOTS; i ++) {
						lbm_value curr = glob_env[i];
						while (lbm_type_of(curr) == LBM_TYPE_CONS) {
							lbm_print_value(output, sizeof(output), lbm_car(curr));
							curr = lbm_cdr(curr);

							commands_printf_lisp("  %s", output);
						}
					}
				}
			} else if (strncmp(str, ":ctxs", 5) == 0) {
				commands_printf_lisp("****** Running contexts ******");
				lbm_running_iterator(print_ctx_info, NULL, NULL);
				commands_printf_lisp("****** Blocked contexts ******");
				lbm_blocked_iterator(print_ctx_info, NULL, NULL);
			} else if (strncmp(str, ":symbols", 8) == 0) {
				if (pause_eval(0, 1000)) {
					lbm_symrepr_name_iterator(sym_it);
					commands_printf_lisp(" ");
				}
			} else if (strncmp(str, ":reset", 6) == 0) {
				commands_printf_lisp(lispif_restart(true, flash_helper_code_size(CODE_IND_LISP) > 0, true) ?
						"Reset OK\n\n" : "Reset Failed\n\n");
			} else if (strncmp(str, ":pause", 6) == 0) {
				if (pause_eval(30, 1000)) {
					commands_printf_lisp("Evaluator paused\n");
				}
			} else if (strncmp(str, ":continue", 9) == 0) {
				lbm_continue_eval();
			} else if (strncmp(str, ":undef", 6) == 0) {
				if (pause_eval(30, 1000)) {
					char *sym = str + 7;
					commands_printf_lisp("undefining: %s", sym);
					commands_printf_lisp("%s", lbm_undefine(sym) ? "Cleared bindings" : "No definition found");
					lbm_continue_eval();
				}
			} else if (strncmp(str, ":verb", 5) == 0) {
				static bool verbose_now = false;
				verbose_now = !verbose_now;
				lbm_set_verbose(verbose_now);
				commands_printf_lisp("Verbose errors %s", verbose_now ? "Enabled" : "Disabled");
			} else if (strncmp(str, ":state", 6) == 0) {
				lbm_uint state = lbm_get_eval_state();
				switch (state) {
					case EVAL_CPS_STATE_DEAD:
						commands_printf_lisp("DEAD\n");
						break;
					case EVAL_CPS_STATE_PAUSED:
						commands_printf_lisp("PAUSED\n");
						break;
					case EVAL_CPS_STATE_NONE:
						commands_printf_lisp("NO STATE\n");
						break;
					case EVAL_CPS_STATE_RUNNING:
						commands_printf_lisp("RUNNING\n");
						break;
					case EVAL_CPS_STATE_KILL:
						commands_printf_lisp("KILLING\n");
						break;
				}
			} else {
				if (repl_buffer) {
					lispif_unlock_lbm();
					break;
				}

				if (pause_eval(30, 1000)) {
					repl_buffer = lbm_malloc_reserve(len);
					if (repl_buffer) {
						memcpy(repl_buffer, data, len);
						lbm_create_string_char_channel(&string_tok_state, &string_tok, repl_buffer);
						repl_cid = lbm_load_and_eval_expression(&string_tok);
						repl_cid_for_buffer = repl_cid;
						lbm_continue_eval();

						if (reply_func != NULL) {
							repl_time = xTaskGetTickCount();
						} else {
							repl_cid = -1;
						}
					} else {
						commands_printf_lisp("Not enough memory");
					}
				} else {
					commands_printf_lisp("Could not pause");
				}
			}
			lispif_unlock_lbm();
		} else {
			commands_printf_lisp("LispBM is not running");
		}
	} break;

	case COMM_LISP_STREAM_CODE: {
		int32_t ind = 0;
		int32_t offset = buffer_get_int32(data, &ind);
		int32_t tot_len = buffer_get_int32(data, &ind);
		int8_t restart = data[ind++];

		static int32_t offset_last = -1;
		static int16_t result_last = -1;

		if (offset == 0) {
			if (!lisp_thd_running) {
				lispif_restart(true, restart == 2 ? true : false, true);
			} else if (restart == 1) {
				lispif_restart(true, false, true);
			} else if (restart == 2) {
				lispif_restart(true, true, true);
			}
		}

		int32_t send_ind = 0;
		uint8_t send_buffer[50];
		send_buffer[send_ind++] = packet_id;
		buffer_append_int32(send_buffer, offset, &send_ind);

		if (offset_last == offset) {
			buffer_append_int16(send_buffer, result_last, &send_ind);
			reply_func(send_buffer, ind);
			break;
		}

		offset_last = offset;

		if (!lisp_thd_running) {
			result_last = -1;
			offset_last = -1;
			buffer_append_int16(send_buffer, result_last, &send_ind);
			reply_func(send_buffer, ind);
			break;
		}

		if (offset == 0) {
			if (string_tok_valid) {
				int timeout = 1500;
				while (!buffered_tok_state.reader_closed) {
					lbm_channel_writer_close(&buffered_string_tok);
					vTaskDelay(1 / portTICK_PERIOD_MS);
					timeout--;
					if (timeout == 0) {
						break;
					}
				}

				if (timeout == 0) {
					result_last = -2;
					offset_last = -1;
					buffer_append_int16(send_buffer, result_last, &send_ind);
					commands_printf_lisp("Reader not closing");
					reply_func(send_buffer, ind);
					break;
				}
			}

			lispif_lock_lbm();

			if (!pause_eval(30, 1000)) {
				lispif_unlock_lbm();
				result_last = -3;
				offset_last = -1;
				buffer_append_int16(send_buffer, result_last, &send_ind);
				commands_printf_lisp("Could not pause");
				reply_func(send_buffer, ind);
				break;
			}

			lbm_create_buffered_char_channel(&buffered_tok_state, &buffered_string_tok);
			string_tok_valid = true;

			if (lbm_load_and_eval_program(&buffered_string_tok, "main-s") <= 0) {
				lispif_unlock_lbm();
				result_last = -4;
				offset_last = -1;
				buffer_append_int16(send_buffer, result_last, &send_ind);
				commands_printf_lisp("Could not start eval");
				reply_func(send_buffer, ind);
				break;
			}

			lbm_continue_eval();
			lispif_unlock_lbm();
		}

		if (!string_tok_valid) {
			result_last = -15;
			buffer_append_int16(send_buffer, result_last, &send_ind);
			commands_printf_lisp("Tokenizer Invalid");
			reply_func(send_buffer, ind);
			break;
		}

		int32_t written = 0;
		int timeout = 1500;
		while (ind < (int32_t)len) {
			int ch_res = lbm_channel_write(&buffered_string_tok, (char)data[ind]);

			if (ch_res == CHANNEL_SUCCESS) {
				ind++;
				written++;
				timeout = 0;
			} else if (ch_res == CHANNEL_READER_CLOSED) {
				break;
			} else {
				vTaskDelay(1 / portTICK_PERIOD_MS);
				timeout--;
				if (timeout == 0) {
					break;
				}
			}
		}

		if (ind == (int32_t)len) {
			if ((offset + written) == tot_len) {
				lbm_channel_writer_close(&buffered_string_tok);
				string_tok_valid = false;
				offset_last = -1;
				commands_printf_lisp("Stream done, starting...");
			}

			result_last = 0;
			buffer_append_int16(send_buffer, result_last, &send_ind);
		} else {
			if (timeout == 0) {
				result_last = -5;
				offset_last = -1;
				buffer_append_int16(send_buffer, result_last, &send_ind);
				commands_printf_lisp("Stream timed out");
			} else {
				result_last = -6;
				offset_last = -1;
				buffer_append_int16(send_buffer, result_last, &send_ind);
				commands_printf_lisp("Stream closed");
			}
		}

		reply_func(send_buffer, send_ind);
	} break;

	case COMM_LISP_RMSG: {
		lispif_process_rmsg(data[0], data + 1, len - 1);
	} break;

	default:
		break;
	}
}

static void done_callback(eval_context_t *ctx) {
	lbm_cid cid = ctx->id;
	lbm_value t = ctx->r;

	if (cid == repl_cid) {
		if (UTILS_AGE_S(repl_time) < 0.5) {
			char output[128];
			lbm_print_value(output, sizeof(output), t);
			commands_printf_lisp("> %s", output);
		} else {
			repl_cid = -1;
		}
	}

	if (cid == repl_cid_for_buffer && repl_buffer) {
		lbm_free(repl_buffer);
		repl_buffer = 0;
	}
}

bool lispif_restart(bool print, bool load_code, bool load_imports) {
	bool res = false;

	restart_cnt++;
	prof_running = false;
	string_tok_valid = false;

	if (prof_running) {
		prof_running = false;
		esp_timer_stop(prof_timer);
	}

	char *code_data = (char*)flash_helper_code_data_ptr(CODE_IND_LISP);
	int32_t code_len = flash_helper_code_size(CODE_IND_LISP);

	if (!load_code || (code_data != 0 && code_len > 0)) {
		lispif_disable_all_events();

		if (!lisp_thd_running) {
			lbm_init(heap, heap_size,
					memory_array, mem_size,
					bitmap_array, bitmap_size,
					GC_STACK_SIZE,
					PRINT_STACK_SIZE,
					extension_storage, EXTENSION_STORAGE_SIZE + USER_EXTENSION_STORAGE_SIZE);
			lbm_eval_init_events(20);

			lbm_set_timestamp_us_callback(timestamp_callback);
			lbm_set_usleep_callback(sleep_callback);
			lbm_set_printf_callback(commands_printf_lisp);
			lbm_set_ctx_done_callback(done_callback);
			xTaskCreatePinnedToCore(eval_thread, "lbm_eval", 3072, NULL, 6, NULL, tskNO_AFFINITY);

			lisp_thd_running = true;
		} else {
			lbm_reset_eval();
			while (lbm_get_eval_state() != EVAL_CPS_STATE_RESET) {
				lbm_reset_eval();
				vTaskDelay(1 / portTICK_PERIOD_MS);
			}

			lbm_init(heap, heap_size,
					memory_array, mem_size,
					bitmap_array, bitmap_size,
					GC_STACK_SIZE,
					PRINT_STACK_SIZE,
					extension_storage, EXTENSION_STORAGE_SIZE + USER_EXTENSION_STORAGE_SIZE);
			lbm_eval_init_events(20);
		}

		lbm_pause_eval();
		while (lbm_get_eval_state() != EVAL_CPS_STATE_PAUSED) {
			lbm_pause_eval();
			vTaskDelay(1 / portTICK_PERIOD_MS);
		}

		lispif_load_vesc_extensions();
		for (int i = 0;i < EXT_LOAD_CALLBACK_LEN;i++) {
			if (ext_load_callbacks[i] == 0) {
				break;
			}

			ext_load_callbacks[i]();
		}

		lbm_set_dynamic_load_callback(lispif_vesc_dynamic_loader);

		int code_chars = 0;
		if (code_data) {
			code_chars = strnlen(code_data, code_len);
		}

		// Load imports
		if (load_imports) {
			if (code_len > code_chars + 3) {
				int32_t ind = code_chars + 1;
				uint16_t num_imports = buffer_get_uint16((uint8_t*)code_data, &ind);

				if (num_imports > 0 && num_imports < 500) {
					for (int i = 0;i < num_imports;i++) {
						char *name = code_data + ind;
						ind += strlen(name) + 1;
						int32_t offset = buffer_get_int32((uint8_t*)code_data, &ind);
						int32_t len = buffer_get_int32((uint8_t*)code_data, &ind);

						lbm_value val;
						if (lbm_share_array(&val, code_data + offset, len)) {
							lbm_define(name, val);
						}
					}
				}
			}
		}

		if (code_data == 0) {
			code_data = (char*)flash_helper_code_data_raw(CODE_IND_LISP);
		}

		const_heap_max_ind = 0;
		const_heap_ptr = (lbm_uint*)(code_data + code_len + 16);
		const_heap_ptr = (lbm_uint*)((uint32_t)const_heap_ptr & 0xFFFFFFF4);
		if ((((uint32_t)code_data + flash_helper_code_size_raw(CODE_IND_LISP))) > (uint32_t)const_heap_ptr) {
			uint32_t const_heap_len = ((uint32_t)code_data + flash_helper_code_size_raw(CODE_IND_LISP)) - (uint32_t)const_heap_ptr;
			lbm_const_heap_init(const_heap_write, &const_heap, (lbm_uint*)const_heap_ptr, const_heap_len);
		}

		if (load_code) {
			if (print) {
				commands_printf_lisp("Parsing %d characters", code_chars);
			}

			lbm_create_string_char_channel(&string_tok_state, &string_tok, code_data);
			lbm_load_and_eval_program_incremental(&string_tok, "main-u");
		}

		lbm_continue_eval();

		res = true;
	}

	if (repl_buffer) {
		lbm_free(repl_buffer);
		repl_buffer = 0;
	}

	return res;
}

void lispif_add_ext_load_callback(void (*p_func)(void)) {
	for (int i = 0;i < EXT_LOAD_CALLBACK_LEN;i++) {
		if (ext_load_callbacks[i] == 0 || ext_load_callbacks[i] == p_func) {
			ext_load_callbacks[i] = p_func;
			break;
		}
	}
}

static uint32_t timestamp_callback(void) {
	TickType_t t = xTaskGetTickCount();
	return (uint32_t) ((1000 / portTICK_PERIOD_MS) * t);
}

static void sleep_callback(uint32_t us) {
	TickType_t t = us / (portTICK_PERIOD_MS * 1000);
	if (t == 0) {
		t = 1;
	}
	vTaskDelay(t);
}

static bool const_heap_write(lbm_uint ix, lbm_uint w) {
	if (ix > const_heap_max_ind) {
		const_heap_max_ind = ix;
	}

	if (const_heap_ptr[ix] == w) {
		return true;
	}

	uint32_t offset = (uint32_t)const_heap_ptr - (uint32_t)flash_helper_code_data_raw(CODE_IND_LISP) + sizeof(lbm_uint) * ix;
	flash_helper_write_code(CODE_IND_LISP, offset, (uint8_t*)&w, sizeof(lbm_uint), (const_heap_max_ind - ix) * sizeof(lbm_uint));

	if (const_heap_ptr[ix] != w) {
		return false;
	}

	return true;
}

static void eval_thread(void *arg) {
	(void)arg;
	eval_task = xTaskGetCurrentTaskHandle();
	lbm_run_eval();
	lisp_thd_running = false;
	vTaskDelete(NULL);
}

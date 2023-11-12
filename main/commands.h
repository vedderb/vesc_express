/*
	Copyright 2022 Benjamin Vedder      benjamin@vedder.se
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

#ifndef MAIN_COMMANDS_H_
#define MAIN_COMMANDS_H_

#include <stdint.h>
#include <stdbool.h>

// Functions
void commands_init(void);
void commands_process_packet(
	unsigned char *data, unsigned int len,
	void (*reply_func)(unsigned char *data, unsigned int len)
);
void commands_send_packet(unsigned char *data, unsigned int len);
void commands_send_packet_can_last(unsigned char *data, unsigned int len);
int commands_printf(const char *format, ...);
int commands_printf_lisp(const char *format, ...);
void commands_init_plot(char *namex, char *namey);
void commands_plot_add_graph(char *name);
void commands_plot_set_graph(int graph);
void commands_send_plot_points(float x, float y);
void commands_send_app_data(unsigned char *data, unsigned int len);

// TODO: remove this
// Rasmus debug stuff
typedef void (*send_func_t)(unsigned char *, unsigned int);
void commands_start_send_func_overwrite(
	void (*new_send_func)(unsigned char *data, unsigned int len)
);
void commands_restore_send_func();
send_func_t commands_get_send_func();
void commands_store_send_func();

// source: https://stackoverflow.com/a/5897216/15507414
#define VA_ARGS(...) , ##__VA_ARGS__

extern volatile send_func_t stored_send_func;
#define stored_printf(fmt, ...)                                                \
	{                                                                          \
		if (stored_send_func) {                                                \
			commands_start_send_func_overwrite(stored_send_func);              \
			commands_printf(fmt VA_ARGS(__VA_ARGS__));                         \
			commands_restore_send_func(stored_send_func);                      \
		}                                                                      \
	}

void commands_print_thread();

#endif /* MAIN_COMMANDS_H_ */

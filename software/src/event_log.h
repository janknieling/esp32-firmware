/* esp32-firmware
 * Copyright (C) 2020-2021 Erik Fleckstein <erik@tinkerforge.com>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the
 * Free Software Foundation, Inc., 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

#pragma once

#include <stdarg.h>
#include <mutex>

#include <Arduino.h>

#include "ringbuffer.h"
#include "malloc_tools.h"

// Length of a timestamp with two spaces at the end. For example "2022-02-11 12:34:56,789"
// Also change in frontend when changing here!
#define TIMESTAMP_LEN 25

#ifdef EVENT_LOG_PREFIX

static const char *logger_prefix = EVENT_LOG_PREFIX ": ";
static size_t logger_prefix_len = strlen(logger_prefix);

#define vprintfln(...) vprintfln_prefixed(logger_prefix, logger_prefix_len, __VA_ARGS__)
#define printfln(...) printfln_prefixed(logger_prefix, logger_prefix_len, __VA_ARGS__)

#define vprintfln_plain(...) vprintfln_prefixed(nullptr, 0, __VA_ARGS__)
#define printfln_plain(...) printfln_prefixed(nullptr, 0, __VA_ARGS__)

#endif

class EventLog
{
public:
    std::mutex event_buf_mutex;
    TF_Ringbuffer<char,
                  10000,
                  uint32_t,
#if defined(BOARD_HAS_PSRAM)
                  malloc_psram,
#else
                  malloc_32bit_addressed,
#endif
                  heap_caps_free> event_buf;

    void pre_init();
    void pre_setup();
    void post_setup();

    void write(const char *buf, size_t len);

    int vprintfln_prefixed(const char *prefix, size_t prefix_len, const char *fmt, va_list args);
    [[gnu::format(__printf__, 4, 5)]] int printfln_prefixed(const char *prefix, size_t prefix_len, const char *fmt, ...);

#ifndef EVENT_LOG_PREFIX
    int vprintfln(const char *fmt, va_list args)
    {
        return vprintfln_prefixed(nullptr, 0, fmt, args);
    }

    [[gnu::format(__printf__, 2, 3)]] int printfln(const char *fmt, ...)
    {
        va_list args;
        va_start(args, fmt);
        int result = vprintfln_prefixed(nullptr, 0, fmt, args);
        va_end(args);

        return result;
    }
#endif

    #define tf_dbg(fmt, ...) printfln("[%s:%d] " fmt, __FILE__, __LINE__ __VA_OPT__(,) __VA_ARGS__)

    void drop(size_t count);

    void register_urls();

    void get_timestamp(char buf[TIMESTAMP_LEN + 1]);

    bool sending_response = false;
};

// Make global variable available everywhere because it is not declared in modules.h.
// Definition is in event_log.cpp.
extern EventLog logger;

// To capture ESP-IDF log messages, use
// esp_log_set_vprintf(tf_event_log_printf);
// If this is in a c (not cpp) file, also add
// extern int tf_event_log_printf(const char *fmt, va_list args);
// instead of including event_log.h (a C++ header)
extern "C" int tf_event_log_printf(const char *fmt, va_list args);

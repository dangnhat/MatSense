/**
 * @file ies_debug.c
 * @author  Nhat Pham <nhat.pham@colorado.edu>.
 * @version 1.0
 * @date 21-Sep-2018
 * @brief This contains marcos and functions for debugging and notification.
 */

#include <stdio.h>
#include <stdarg.h>
#include "ies_debug.h"

#define MAX_BUF_SIZE 256
static uint16_t cur_ptr = 0;
char ies_debug_buffer[MAX_BUF_SIZE];

void ies_printf(const char *format, ...) {
  va_list arg;
  int num_chars_written;
  int index;

  va_start(arg, format);
  num_chars_written = vsnprintf(&ies_debug_buffer[cur_ptr], MAX_BUF_SIZE - cur_ptr, format,
      arg);
  va_end(arg);

  cur_ptr += num_chars_written;

  for (index = 0; index < cur_ptr; index++) {
    if (ies_debug_buffer[index] == '\n') {
      ies_flush();
      break;
    }
  }

  return;
}

void ies_flush(void) {
  Display_clearLine(DISPLAY_HANDLE, IES_RESULT2);
  Display_printf(DISPLAY_HANDLE, IES_RESULT2, 0, ies_debug_buffer);
  cur_ptr = 0;
}


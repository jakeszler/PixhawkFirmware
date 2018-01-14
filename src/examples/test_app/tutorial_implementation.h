/*
 * Copyright (C) 2014 Swift Navigation Inc.
 * Contact: Colin Beighley <colin@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

/*
 * tutorial_implementation contains functions and definitions that are
 * implementation specific to this tutorial, to keep main.c as simple as possible.
 */
#include <stdint.h>


#define DO_EVERY(n, cmd) do { \
  static u32 do_every_count = 0; \
  if (do_every_count % (n) == 0) { \
    cmd; \
  } \
  do_every_count++; \
} while(0)

/* FIFO functions */
uint8_t fifo_empty(void);
uint8_t fifo_full(void);
uint8_t fifo_write(char c);
uint8_t fifo_read_char(char *c);
uint32_t fifo_read(uint8_t *buff, uint32_t n, void *context);
void printall(void);
/* UART functions */
void usarts_setup(void);

/* LED functions */
void leds_set(void);
void leds_unset(void);
void leds_toggle(void);
void leds_setup(void);
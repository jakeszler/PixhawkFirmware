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
 * tutorial_implementation contains functions and definitions that are implementation
 * specific to this tutorial, to keep main.c as simple as possible.
 */


#include "tutorial_implementation.h"



/*
 * FIFO to hold received UART bytes before libsbp parses them.
 */
#define FIFO_LEN 512
char sbp_msg_fifo[FIFO_LEN];
uint16_t head = 0;
uint16_t tail = 0;

/* Return 1 if true, 0 otherwise. */
uint8_t fifo_empty(void){
  if (head == tail)
    return 1;
  return 0;
}

/*
 * Append a character to our SBP message fifo.
 * Returns 1 if char successfully appended to fifo.
 * Returns 0 if fifo is full.
 */
uint8_t fifo_write(char c){
  if (fifo_full())
    return 0;

  sbp_msg_fifo[tail] = c;
  tail = (tail+1) % FIFO_LEN;
  return 1;
}

/*
 * Read 1 char from fifo.
 * Returns 0 if fifo is empty, otherwise 1.
 */
uint8_t fifo_read_char(char *c) {
  if (fifo_empty())
    return 0;

  *c = sbp_msg_fifo[head];
  //PX4_INFO("fiforead: %c", *c);
  head = (head+1) % FIFO_LEN;
  return 1;
}

/*
 * Read arbitrary number of chars from FIFO. Must conform to
 * function definition that is passed to the function
 * sbp_process().
 * Returns the number of characters successfully read.
 */
uint32_t fifo_read(uint8_t *buff, uint32_t n, void *context) {
  int i;
  for (i=0; i<n; i++)
    if (!fifo_read_char((char *)(buff + i)))
      break;
  return i;
}
//used for debugging
// void printall()
// {
//   for(int i = 0; i <FIFO_LEN; i++)
//   {
//     PX4_INFO("%02X",sbp_msg_fifo[i]);
//   }
// }

/* Return 1 if true, 0 otherwise. */
uint8_t fifo_full(void){
  if (((tail+1)%FIFO_LEN) == head) {
    return 1;
  }
  return 0;
}
//add (fifo_write to use it)


void usarts_setup(void){

  
}

void leds_set(void){
 
}

void leds_unset(void){
  
}

void leds_toggle(void){

}

void leds_setup(void)
{

}
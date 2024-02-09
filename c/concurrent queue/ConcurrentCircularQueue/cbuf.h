/**
 * @file cbuf.h
 * @brief circular buffer that performs enqueuing and dequeuing concurrently
 * @author boo
 */

#ifndef CIRCULAR_BUFFER_H
#define CIRCULAR_BUFFER_H

#include <stdio.h>

//! default circular buffer size
#define CBUF_DEFAULT_BUF_SIZE (32)

/**
 * @brief function that initializes circular buffer
 * @return 0 if initialization done. Otherwise, return error code.
 */
int cbuf_init(void);

/**
 * @brief function that de-initializes circular buffer
 * @return 0 if de-initialization done. Otherwise, return error code.
 */
int cbuf_deinit(void);

/**
 * @brief function that enqueues data into circular buffer
 * @pre circular queue must be initialization
 * @param [in] elem - element to be enqueued
 * @return 0 if enqueued. Otherwise, return error code.
 */
int cbuf_enqueue(int elem);

/**
 * @brief function that dequeues data frm circular buffer
 * @pre circular queue must be initialization
 * @param [out] elem - element to be dequeued
 * @return 0 if dequeued. Otherwise, return error code.
 */
int cbuf_dequeue(int *elem);

/**
 * @brief function that prints circular buffer's content as a list
 * @note it is not part of circular buffer design, but implemented to view the buffer,
 *       in output, H indicates head's current location, T indicates tail's current location.
 */
void cbuf_print(void);

#endif

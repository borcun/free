/**
 * @file lfcb.h
 * @brief lock-free circular buffer, including enqueuing and dequeuing operations
 */

#ifndef LOCK_FREE_CIRCULAR_BUFFER_H
#define LOCK_FREE_CIRCULAR_BUFFER_H

//! lock-free buffer size
#define LFCB_BUFFER_SIZE (32)

/**
 * @brief function that initializes the buffer
 * @return 0
 */
int lfcb_init(void);

/**
 * @brief function that enqueues an element to the buffer
 * @param [in] elem - element to be enqueued
 * @return 0 if the element is enqueued. otherwise, return -1.
 */
int lfcb_enqueue(int elem);

/**
 * @brief function that dequeues an element from the buffer
 * @param [out] elem - element to be dequeued
 * @return 0 if the element is dequeued. otherwise, return -1.
 */
int lfcb_dequeue(int &elem);

/**
 * @brief function that prints buffer content
 * @return
 */
void lfcb_print(void);

#endif

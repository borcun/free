/**
 * @file cbufTest.h
 * @brief circular buffer test that performs enqueuing and dequeuing operations
 * @author boo
 */

#ifndef CIRCULAR_BUFFER_TEST_H
#define CIRCULAR_BUFFER_TEST_H

#include "cbuf.h"

int cbufTest_init(void);
int cbufTest_transaction(void);
int cbufTest_deinit(void);

#endif

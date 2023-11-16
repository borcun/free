#ifndef MACRO_H
#define MACRO_H

#include <stdio.h>

// macro that sums two numbers miserably
#define MISERABLE_SUM_MACRO(x, y) x + y
// macro that multiplies two numbers miserably
#define MISERABLE_MUL_MACRO(x, y) x * y

// macro that sums two numbers
#define CORRECT_SUM_MACRO(x, y) ((x) + (y))
// macro that multiples two numbers
#define CORRECT_MUL_MACRO(x, y) ((x) * (y))

#endif

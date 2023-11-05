#ifndef MISERABLE_H
#define MISERABLE_H

#include <stdio.h>

/// @attention the below four macros are used for dangerous macro scenario
// macro that sums two numbers miserably
#define MISERABLE_SUM_MACRO(x, y) x + y
// macro that multiplies two numbers miserably
#define MISERABLE_MUL_MACRO(x, y) x * y
// macro that sums two numbers
#define CORRECT_SUM_MACRO(x, y) ((x) + (y))
// macro that multiples two numbers
#define CORRECT_MUL_MACRO(x, y) ((x) * (y))

/// @attention the struct includes padding, used to indicate padding problem
typedef struct {
   uint8_t uuid;
   uint32_t code;
   uint16_t len;
} StructWPadding;

/// @attention the struct not includes padding
#pragma pack(1)
typedef struct {
   uint8_t uuid;
   uint32_t code;
   uint16_t len;
} StructWOPadding;

// functions that are used to set and display structures with and w/o padding
void setStructWithPadding(StructWPadding *swp, const uint8_t *data, const uint8_t size);
void setStructWithoutPadding(StructWOPadding *swop, const uint8_t *data, const uint8_t size);
void displayStructWithPadding(const StructWPadding swp);
void displayStructWithoutPadding(const StructWOPadding swop);

// functions that get user name in turn as non-const pointer and const pointer
/// @attention first function returns dangling pointer, but second one does not
char *getUsername(void);
const char * const getUsername_s(void);

// call-by-value & call-by-reference test functions
void callByValue(int num);
void callByRef(int *num);
void memAllocP(int *ptr);
void memAllocPP(int **ptr);

// function that initializes/deinitializes miserable library
int init(void);
void deinit(void);

#endif

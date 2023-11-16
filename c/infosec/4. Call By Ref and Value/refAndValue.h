#ifndef REF_AND_VALUE
#define REF_AND_VALUE

#include <stdio.h>

// these functions are for demos of call by value and call by reference
void callByValue(int num);
void callByRef(int *num);

// these functions depict that call by reference is actually performed as call by value
void memAllocP(int *ptr);
void memAllocPP(int **ptr);

#endif

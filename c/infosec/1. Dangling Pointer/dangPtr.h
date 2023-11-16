#ifndef DANGLING_POINTER_H
#define DANGLING_POINTER_H

#include <stdio.h>

#define DEF_UNAME ("admin")
#define DEF_UNAME_LEN (8)

// function that initializes/deinitializes dangling pointer library
int init(void);
void deinit(void);

/*
 * functions that get user name in turn as non-const pointer and const pointer
 * first one is used to show how dangling pointer exist, other one for preventing it 
 */
char *getUsername(void);
const char * const getUsername_s(void); // secure one

#endif

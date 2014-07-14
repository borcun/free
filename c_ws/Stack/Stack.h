#ifndef STACK_H
#define STACK_H

#include <stdio.h>
#include <stdlib.h>

#define MAX_SIZE 100
#define TRUE 1
#define FALSE 0

// stack structure
typedef struct Stack
{
  int *data;
  int size;
  int top;
} Stack;

typedef struct Stack *StackPtr;

// function that initialize stack
int init(StackPtr stack, int _size);
// function that checks whether stack is empty
int isEmpty(Stack stack);
// function that gets elements count of stack
int size(Stack stack);
// function that pops the top element from stack
int pop(StackPtr stack);
// function that pushs an element into stack
int push(StackPtr stack, int elem);
// function that deallocates memory
void dealloc(Stack stack);
// function that prints content of stack
void print(Stack stack);

#endif

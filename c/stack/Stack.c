#include "Stack.h"

// function that initialize stack
int init(StackPtr stack, int _size)
{
  if(_size < 1 || _size > MAX_SIZE) {
    fprintf(stderr, "%s\n", "invalid size");
    return FALSE;
  } 
  
  if(NULL == (stack->data = (int *)malloc(sizeof(int) * _size))) {
    fprintf(stderr, "%s\n", "memory allocation error");
    return FALSE;
  }

  stack->size = _size;
  stack->top = 0;
}

// function that pops the top element from stack
int pop(StackPtr stack)
{
  if(isEmpty(*stack)) {
    fprintf(stderr, "%s\n", "Stack is empty");
    return FALSE;
  }

  return stack->data[stack->top--];
}

// function that pushs an element into stack
int push(StackPtr stack, int elem)
{
  if(stack->size == MAX_SIZE || stack->top == stack->size) {
    fprintf(stderr, "%s\n", "Stack is full");
    return FALSE;
  }

  stack->data[stack->top++] = elem;

  return TRUE;
}

// function that checks whether stack is empty
int isEmpty(Stack stack)
{
  return stack.top == 0;
}

// function that gets elements count of stack
int size(Stack stack)
{
  return stack.top;
}

// function that deallocates memory
void dealloc(Stack stack)
{
  free(stack.data);
  return;
}

// function that prints content of stack
void print(Stack stack)
{
  int i;

  if(isEmpty(stack)) {
    fprintf(stdout, "%s\n", "Stack is empty");
    return;
  }

  for(i = stack.top -1 ; i >= 0 ; --i)
    printf("%d\n", stack.data[i]);

  return;
}

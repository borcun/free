#include "Stack.h"

int main()
{
  Stack stack;

  init(&stack, 10);

  push(&stack, 9);
  push(&stack, -1);
  push(&stack, 6);

  printf("Push Function\n");
  print(stack);

  pop(&stack);

  printf("\nPop Function\n");
  print(stack);

  printf("\nSize of Stack %d\n", size(stack));

  dealloc(stack);

  return 0;
}

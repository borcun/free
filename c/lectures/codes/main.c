#include <stdio.h>

void swap1(int *ptr1, int *ptr2) {
  *ptr2 = -70;
  *ptr2 = *ptr1;
  *ptr1 = 100;
}

void swap2(int *restrict ptr1, int *restrict ptr2) {
  *ptr2 = -70;
  *ptr2 = *ptr1;
  *ptr1 = 100;
}

int main() {
  int x = 3;
  int y = -7;
  int *xptr = &x;
  int *yptr = &y;
  
  printf("[before] xptr, yptr : %d %d\n", *xptr, *yptr);
  swap2(xptr, yptr);
  printf("[after] xptr, yptr : %d %d\n", *xptr, *yptr);

  return 0;
}

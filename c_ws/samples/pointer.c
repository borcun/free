#include <stdio.h>

void change(int *iptr)
{
  *iptr = 4;
}

int main()
{
  double arr[4] = {2.0, 5.0, 1.0, 3.0};
  double *yptr;

  yptr = &arr[0];

  printf("addr of &arr[0]: %p\n", &arr[0]);
  printf("addr of &arr: %p\n", &arr);
  printf("addr of arr: %p\n", arr);
  printf("yptr: %p\n", yptr);
  printf("*yptr: %f\n\n", *yptr);

  yptr += 2;

  printf("addr of &arr[2]: %p\n", &arr[2]);
  printf("yptr: %p\n", yptr);
  printf("*yptr: %f\n", *yptr);

  return 0;
}

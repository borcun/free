#include <stdio.h>

int main() {
  extern int var;
  extern int con_var;
  int arr[3] = {
    [0] = 1,
    [1] = 2,
    [2] = 4
  };
  
  printf("variable: %d\n", var);
  printf("container variable: %d\n", con_var);
  
  return 0;
}

int var = 10;

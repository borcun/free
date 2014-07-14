#include <stdio.h>

int main()
{
  int i = 20;
  
  do {
    printf("%d ", i);
  }
  while(++i <= 10);
  
  printf("\n");

  return 0;
}

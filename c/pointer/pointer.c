#include <stdio.h>
#include <stdlib.h>

int main()
{
  int i, n, *p, *ptr;

  printf("enter size: ");
  scanf("%d", &n);

  ptr = (int *)malloc(sizeof(int) * n);
  p = ptr;


  for(i=0 ; i < n ; ++i) {
    printf("enter number: ");
    scanf("%d", ptr);
    ++ptr;
  }

  for(i=0 ; i < n ; ++i) {
    printf("ptr[%d]: %d\n", i, *ptr);
    printf("p[%d]: %d\n", i, *p);
    ++ptr;
    ++p;
  }

  return 0;
}

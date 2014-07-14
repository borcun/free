#include <stdio.h>

int main()
{
  int i = 1;
  
  while(1) {
    printf("...., cikmak icin 0 girin: ");
    scanf("%d", &i);
    
    if(i == 0)
      break;
    else
      printf("%d\n", i);
  }

  printf("\n");

  return 0;
}

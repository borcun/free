#include <stdio.h>

int main()
{
  int product = 100;
  
  while(product >= 1) {
    if(product == 100) {
    }
    else {
      if(product % 10 == 0)
        printf("\n");
    }
     
    printf("%3d ", product);   
    product = product - 1;
  }
  
  printf("\n");

  return 0;
}

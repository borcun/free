#include <stdio.h>

int main()
{
  int num1, num2, num3;

  scanf("%d %d %d", &num1, &num2, &num3);

  if(num1 < num2) {
    if(num1 < num3)
      printf("Minimum number %d\n", num1);
    else
      printf("Mininum number %d\n", num3);
  }
  else {
    if(num2 < num3)
      printf("Minimum number %d\n", num2);
    else
      printf("Minimum number %d\n", num3);
  }
  
  return 0;
}

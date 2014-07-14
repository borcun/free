#include <stdio.h>

int main()
{
  int i;
  float f;
  
  f = 3.14;
  
  // cast
  i = (int)f;
  f = (float)i;
  
  printf("%f\n", f);  
  
  return 0;
}

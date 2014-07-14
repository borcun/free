#include <stdio.h>

int main()
{
  int i;
  
  for(i=1 ; i <= 10 ; ++i) {
    switch(i % 2) {
      case 0:
        printf("%d ikinin katidir\n\n", i);
      case 1:
        if(i % 3 == 0)
          printf("%d ucun katidir\n\n", i);
      break;
    }
  }

  return 0;
}

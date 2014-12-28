#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>

void doubleRoll(int d1, int d2)
{
  srand(time(NULL));
  d1 = 1 + rand() % 6;
  sleep(1);
  d2 = 1 + rand() % 6; 

  return;
}

int main()
{  
  int d1 = -1, d2 = -1;

  doubleRoll(d1, d2);

  printf("d1 = %d\nd2 = %d\n", d1, d2);

  return 0;
}

/*

SET counter 1
SET not1, not2, not3

WHILE counter < 3
  PRINT Enter next grade
  
  IF counter = 1 THEN
    READ not1
  ELSE
    IF counter = 2 THEN
      READ not2
    ELSE
      READ not3
  
  SET counter counter + 1
  
CALCULATE (not1 + not2 + not3) / 3
*/

#include <stdio.h>

int main()
{
  int not;
  int sum = 0;
  int counter = 1;
  
  while(counter <= 3) {
    printf("Notu giriniz: ");    
    scanf("%d", &not);

    sum = sum + not;
    
    counter = counter + 1;
  }
  
  printf("Ortalama : %d\n", sum / 3);

  return 0;
}

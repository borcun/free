/**
 * @description: calculator
 * @author: burak orcun ozkablan
 * @date: June 15, 2014
 */

#include <stdio.h>

// main function
int main()
{
  char op = 'x';
  int num1, num2;
  
  while(op != 'q') {    
    printf("Toplama icin +\nCikarma icin -\nCarpma icin *\nBolme icin /\nMod icin %\nCikis icin q basiniz\n");
    scanf(" %c", &op);

    // toplama
    if(op == '+') {
      printf("Toplama islemi icin iki adet sayi giriniz\n");
      scanf("%d %d", &num1, &num2);
      
      printf("%d + %d = %d\n", num1, num2, (num1 + num2));
    }
    
    // cikarma 
    if(op == '-') {
      printf("Cikarma islemi icin iki adet sayi giriniz\n");
      scanf("%d %d", &num1, &num2);

      printf("%d - %d = %d\n", num1, num2, (num1 - num2));    
    }
    
    // ...
  
    if(op == 'q') {
      printf("Cikis\n");
    } // end of if
  } // end of while

  return 0;
}

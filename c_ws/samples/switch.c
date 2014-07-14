#include <stdio.h>

int main()
{
  int i;
  
  printf("i icin bir deger giriniz: ");
  scanf("%d", &i);
  
  switch(i) {
    case 2:
    case 4:
      printf("2 veya 4 girdiniz\n");
    break;
    case 3:
      printf("3 girdiniz\n");
    break;
    case 1:
      printf("1 girdiniz\n");
    break;
    default:
      printf("1,2,3 disinda bir sayi girdiniz\n");
  }
  
  /*
  if(i == 1) {
    printf("1 girdiniz\n");
  }
  else {
    if(i == 2) {
      printf("2 girdiniz\n");
    }
    else {  
      if(i == 3)
        printf("3 girdiniz\n");
      else
        printf("1,2,3 disinda bir sayi girdiniz\n");
    }
  }
  */
    
  return 0;
}

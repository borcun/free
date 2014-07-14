#include <stdio.h>

int main() 
{
  int grade;

  printf("Enter a grade : ");
  scanf("%d", &grade);  

  // not 90'dan buyukse
  if(grade >= 90) {
    printf("A\n");
  }
  else {
    // not 80'den buyukse ve 90'dan kucukse
    if(grade >= 80) {
      printf("B\n");
    }
    else {
      // not 70'den buyuk ve 80'den kucukse
      if(grade >= 70) {
        printf("C\n");
      }
      // not 70'den kucuk ise
      else {
        printf("D\n");
      }
    }
  }

  return 0;
}

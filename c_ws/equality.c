#include <stdio.h>

int main()
{
  char ch1, ch2;
  
  printf("Enter two characters\n");
  
  scanf("%c %c", &ch1, &ch2);
  
  if(ch1 == ch2)
    printf("Two characters are same\n");
  
  if(ch1 != ch2) {
    printf("Two characters are not same\n");
  }
  
  if(ch1 > ch2) {
    printf("%c is bigger than %c\n", ch1, ch2);
  }

  if(ch1 < ch2) {
    printf("%c is smaller than %c\n", ch1, ch2);
  }

  if(ch1 >= ch2) {
    printf("%c is bigger than %c or equals %c\n", ch1, ch2, ch2);
  }

  if(ch1 <= ch2) {
    printf("%c is smaller than %c or equals %c\n", ch1, ch2, ch2);
  }

  return 0;
}

#include <stdio.h>

#define SIZE 32

char *_strcat(char *dst, char *src);

int main(int argc, char **argv)
{
  char dst[SIZE] = "richard ";
  char src[SIZE] = "stallman";

  _strcat(dst, src);
  printf("dst: %s\n", dst);

  return 0;
}

char *_strcat(char *dst, char *src)
{
  while(*(++dst) != '\0');
  while((*dst++ = *src++) != '\0');

  return dst;
}

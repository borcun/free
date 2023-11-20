#include <stdio.h>
#include <string.h>

/*
 * the return type is size_t, because it's also return type of strlen.
 * size_t is return type frequently used in C programming language.
 */
size_t histogram(const char * const str) {
  return NULL == str ? 0 : strlen(str);
}

int main(int argc, char **argv) {
  size_t len = 0;
  
  if (2 != argc) {
    fprintf(stdout, "usage: %s\n", "./int-oflow <word>");
    return -1;
  }

  len = histogram(argv[1]);

  printf("%s: ", argv[1]);
  
  // what if replace greater operator (>) with greater and equal operator (>=)
  while (len-- > 0) {
    printf("%c", '*');
  }

  puts("");

  return 0;
}

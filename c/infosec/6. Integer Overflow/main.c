#include <stdio.h>

int main() {
  unsigned int limit = 10;

  // what if replace greater operator (>) with greater and equal operator (>=)
  while (limit > 0) {
    printf("limit: %d\n", limit);
    --limit;
  }

  return 0;
}

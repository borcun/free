#include <stdio.h>

int main() {
  size_t limit = 10;

  // what if replace greater operator (>) with greater and equal operator (>=)
  while (limit > 0) {
    printf("limit: %zu\n", limit);
    --limit;
  }

  return 0;
}

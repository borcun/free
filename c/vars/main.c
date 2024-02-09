#include "display.h"

extern int gParam;

int main() {
  int x = 3;

  display_param(x);

  printf("gParam in main: %d\n", gParam);

  return 0;
}

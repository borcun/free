#include "display.h"

int gParam = 5;

void display_param(int param) {
  printf("parameters: %d, global param %d\n", param, gParam);
  return;
}

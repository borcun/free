#include "AxeBehaviour.h"
#include <iostream>

AxeBehaviour::~AxeBehaviour() {
}

void AxeBehaviour::useWeapon(void) {
  std::cout << "chopping with an axe" << std::endl;
  return;
}

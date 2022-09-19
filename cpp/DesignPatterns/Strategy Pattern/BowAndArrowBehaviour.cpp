#include "BowAndArrowBehaviour.h"
#include <iostream>

BowAndArrowBehaviour::~BowAndArrowBehaviour() {
}

void BowAndArrowBehaviour::useWeapon(void) {
  std::cout << "shooting an arrow with a bow" << std::endl;
  return;
}

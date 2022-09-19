#include "SwordBehaviour.h"
#include <iostream>

SwordBehaviour::~SwordBehaviour() {
}

void SwordBehaviour::useWeapon(void) {
  std::cout << "swinging with a sword" << std::endl;
  return;
}

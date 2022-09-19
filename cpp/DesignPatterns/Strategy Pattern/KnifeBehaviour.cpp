#include "KnifeBehaviour.h"
#include <iostream>

KnifeBehaviour::~KnifeBehaviour() {
}

void KnifeBehaviour::useWeapon(void) {
  std::cout << "cutting with a knife" << std::endl;
  return;
}

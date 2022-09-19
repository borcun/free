#include "King.h"
#include <iostream>

King::King(const std::string &name) : Warrior(name) {
}

King::~King() {
}

void King::fight(void) {
  std::cout << m_name << " ";
  m_weapon->useWeapon();
  
  return;
}

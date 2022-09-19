#include "Knight.h"
#include <iostream>

Knight::Knight(const std::string &name) : Warrior(name) {
}

Knight::~Knight() {
}

void Knight::fight(void) {
  std::cout << m_name << " ";
  m_weapon->useWeapon();

  return;
}

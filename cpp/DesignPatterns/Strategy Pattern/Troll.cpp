#include "Troll.h"
#include <iostream>

Troll::Troll(const std::string &name) : Warrior(name) {
}

Troll::~Troll() {
}

void Troll::fight(void) {
  std::cout << m_name << " "; 
  m_weapon->useWeapon();

  return;
}

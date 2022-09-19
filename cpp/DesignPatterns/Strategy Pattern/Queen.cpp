#include "Queen.h"
#include <iostream>

Queen::Queen(const std::string &name) : Warrior(name) {
}

Queen::~Queen() {

}

void Queen::fight(void) {
  std::cout << m_name << " ";
  m_weapon->useWeapon();
  
  return;
}

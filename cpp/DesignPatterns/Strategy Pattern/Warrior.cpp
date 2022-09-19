#include "Warrior.h"

Warrior::Warrior(const std::string &name) : m_name(name) {
}

Warrior::~Warrior() {
}
    
void Warrior::setWeapon(WeaponBehaviour *weapon) {
  m_weapon = weapon;
}

#include "Character.h"

void Character::setHealth( const double cHealth ) {
  if( cHealth < 0 ) {
    std::cout << "The health can not be negative" << std::endl;
    return;
  }

  health = cHealth;
  return;
}

void Character::setMaxHealth( const double cMaxHealth ) {
  if( cMaxHealth < 0 ) {
    std::cout << "The maximum health can not be negative" << std::endl;
    return;
  }

  maxHealth = cMaxHealth;
  return;
}

void Character::setArmor( const double cArmor ) {
  if( cArmor < 0 ) {
    std::cout << "The armor can not be negative" << std::endl;
    return;
  }

  armor = cArmor;
  return;
}

void Character::setDamage( const double cDamage ) {
  if( cDamage < 0 ) {
    std::cout << "The damage can not be negative" << std::endl;
    return;
  }

  damage = cDamage;
  return;
}

void Character::setCritChance( const double cCritChance ) {
  if( cCritChance < 0 ) {
    std::cout << "The critical chance can not be negative" << std::endl;
    return;
  }

  critChance = cCritChance;
  return;
}

void Character::setPotionCount( const int cPotionCount ) {
  if( cPotionCount < 0 ) {
    std::cout << "The potion count can not be negative" << std::endl;
    return;
  }

  potionCount = cPotionCount;
  return;
}

double Character::getHealth( void ) const {
  return health;
}

double Character::getMaxHealth( void ) const {
  return maxHealth;
}

double Character::getArmor( void ) const {
  return armor;
}

double Character::getDamage( void ) const {
  return damage;
}

double Character::getCritChance( void ) const {
  return critChance;
}

int Character::getPotionCount( void ) const {
  return potionCount;
}
  
bool Character::alive( void ) const {
  return isAlive;
}

void Character::takeDamage( const double damageAmount ) {
  std::cout << "I took damage: " << damageAmount << std::endl;
  return;
}

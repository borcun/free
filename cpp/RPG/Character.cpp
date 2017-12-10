#include "Character.h"

Character::Character( void ) {
  health = CHARACTER_MAX_HEALTH_VALUE;
  maxHealth = CHARACTER_MAX_HEALTH_VALUE;
  armor = CHARACTER_ARMOR_VALUE;
  damage = 0.0;
  critChance = 0.0;
  potionCount = 0;
  isAlive = true;
}

Character::~Character() {

}

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

double Character::attack( void ) {
  return CHARACTER_ATTACK_VALUE;
}

void Character::takeDamage( const double damageAmount ) {
  if( armor <= 0 ) {
    if( health > 0 ) {
      health -= damageAmount;
    }
    else {
      isAlive = false;
    }
  }
  else {
    armor -= damageAmount;

    if( armor < 0 ) {
      health += armor;
      armor = 0;

      if( health < 0 ) {
	isAlive = false;
      }
    }
  }
    
  return;
}

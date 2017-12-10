#include "Character.h"

Character::Character( void ) {
  health = 0;
  maxHealth = 0;
  armor = 0;
  damage = 0;
  critChance = 0.0;
  potionCount = 0;
  isAlive = true;
}

Character::~Character() {

}

void Character::setHealth( const double cHealth ) {
  if( cHealth < 0 ) {
    cout << "The health can not be negative" << endl;
    return;
  }

  health = cHealth;
  return;
}

void Character::setMaxHealth( const double cMaxHealth ) {
  if( cMaxHealth < 0 ) {
    cout << "The maximum health can not be negative" << endl;
    return;
  }

  maxHealth = cMaxHealth;
  return;
}

void Character::setArmor( const double cArmor ) {
  if( cArmor < 0 ) {
    cout << "The armor can not be negative" << endl;
    return;
  }

  armor = cArmor;
  return;
}

void Character::setDamage( const double cDamage ) {
  if( cDamage < 0 ) {
    cout << "The damage can not be negative" << endl;
    return;
  }

  damage = cDamage;
  return;
}

void Character::setCritChance( const double cCritChance ) {
  if( cCritChance < 0 ) {
    cout << "The critical chance can not be negative" << endl;
    return;
  }

  critChance = cCritChance;
  return;
}

void Character::setPotionCount( const int cPotionCount ) {
  if( cPotionCount < 0 ) {
    cout << "The potion count can not be negative" << endl;
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
  return damage;
}

void Character::takeDamage( const double damageAmount ) {
  if( armor <= 0.0 ) {
    if( health > 0.0 ) {
      health -= damageAmount;

      if( health <= 0.0 ) {
	isAlive = false;
      }
    }
    else {
      isAlive = false;
    }
  }
  else {
    armor -= ( damageAmount / 50.0 );

    if( armor <= 0.0 ) {
      health += armor;
      armor = 0.0;

      if( health <= 0.0 ) {
	isAlive = false;
      }
    }
  }
    
  return;
}

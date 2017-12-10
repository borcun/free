#include "Enemy.h"

void Enemy::setExperienceAmount( const int eExperienceAmount ) {
  if( eExperienceAmount < 0 ) {
    std::cout << "Enemy experience amount can not be negative" << std::endl;
    return;
  }

  experienceAmount = eExperienceAmount;
  return;
}

void Enemy::setPotionChance( const double ePotionChance ) {
  if( ePotionChance < 0 ) {
    std::cout << "Enemy potion chance can not be negative" << std::endl;
    return;
  }

  potionChance = ePotionChance;
  return;
}

int Enemy::getExperienceAmount( void ) const {
  return experienceAmount;
}

double Enemy::getPotionChance( void ) const {
  return potionChance;
}

void Enemy::information( void ) {
  std::cout << "Enemy Object" << std::endl;
  return;
}

void Enemy::drinkPotion( void ) {
  std::cout << "Enemy drunk potion" << std::endl;
  return;
}

void Enemy::decideAction( void ) {
  std::cout << "Enemy decided action" << std::endl;
  return;
}

#include "Enemy.h"

Enemy::Enemy( void ) : Character() {
  experienceAmount = 0;

  setPotionCount( ENEMY_POTION_CHANCE );
}

Enemy::~Enemy() {

}

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
  std::cout << "Experience Amount: " << experienceAmount << std::endl;
  std::cout << "Potion Chance: " << potionChance << std::endl;
  
  return;
}

void Enemy::drinkPotion( void ) {
  if( 0 == getPotionCount() ) {
    std::cout << "There is no potion chance for enemy" << std::endl;
  }
  else {
    std::cout << "Enemy drunk potion" << std::endl;
    setPotionCount( getPotionCount() - 1 );

    int health = ( int ) getHealth() + CHARACTER_POTION_VALUE;

    health = health % ( CHARACTER_MAX_HEALTH_VALUE + 1 );
    setHealth( ( double ) health );
  }
  
  return;
}

void Enemy::decideAction( void ) {
  std::cout << "Enemy decided action" << std::endl;
  return;
}

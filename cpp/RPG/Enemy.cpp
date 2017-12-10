#include "Enemy.h"

Enemy::Enemy( void ) : Character() {
  setExperienceAmount( randNum( ENEMY_STARTUP_MIN_XP, ENEMY_STARTUP_MAX_XP ) );
  setPotionCount( ENEMY_POTION_CHANCE );
  setHealth( randNum( ENEMY_STARTUP_MIN_HEALTH, ENEMY_STARTUP_MAX_HEALTH ) );
  setMaxHealth( getHealth() );
  setArmor( ENEMY_STARTUP_ARMOR );
  setExperienceAmount( randNum( ENEMY_STARTUP_MIN_XP, ENEMY_STARTUP_MAX_XP ) );
  setDamage( ENEMY_STARTUP_DAMAGE );
  setPotionCount( randNum( ENEMY_STARTUP_MIN_POTION_COUNT, ENEMY_STARTUP_MAX_POTION_COUNT ) );
}

Enemy::~Enemy() {

}

void Enemy::setExperienceAmount( const int eExperienceAmount ) {
  if( eExperienceAmount < 0 ) {
    cout << "Enemy experience amount can not be negative" << endl;
    return;
  }

  experienceAmount = eExperienceAmount;
  return;
}

int Enemy::getExperienceAmount( void ) const {
  return experienceAmount;
}

void Enemy::information( void ) {
  cout << "Enemy Object" << endl;
  cout << "Experience Amount: " << experienceAmount << endl;
  
  return;
}

void Enemy::drinkPotion( void ) {
  if( 0 == getPotionCount() ) {
    cout << "There is no potion chance for enemy" << endl;
  }
  else {
    cout << "Enemy drunk potion" << endl;
    setPotionCount( getPotionCount() - 1 );

    double health = getHealth() + ( double ) CHARACTER_POTION_VALUE;

    if( health > CHARACTER_MAX_HEALTH_VALUE ) {
      health = ( double ) CHARACTER_MAX_HEALTH_VALUE;
    }
    
    setHealth( health );
  }
  
  return;
}

void Enemy::decideAction( void ) {
  cout << "Enemy decided action" << endl;
  return;
}

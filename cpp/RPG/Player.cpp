#include "Player.h"

Player::Player( void ) : Character() {
  goldCount = 0;
  level = 0;
  experience = 0;
  toLevelUp = 0;

  setPotionCount( PLAYER_POTION_CHANCE );
}

Player::~Player() {

}

void Player::setGoldCount( const int pGoldCount ) {
  if( pGoldCount < 0 ) {
    std::cout << "Player gold count can not be negative" << std::endl;
    return;
  }

  goldCount = pGoldCount;
  return;
}

void Player::setLevel( const int pLevel ) {
  if( pLevel < 0 ) {
    std::cout << "Player level can not be negative" << std::endl;
    return;
  }

  level = pLevel;
  return;
}

void Player::setExperience( const int pExperience ) {
  if( pExperience < 0 ) {
    std::cout << "Player experience can not be negative" << std::endl;
    return;
  }

  experience = pExperience;
  return;
}

void Player::setToLevelUp( const int pToLevelUp ) {
  if( pToLevelUp < 0 ) {
    std::cout << "Player to level up can not be negative" << std::endl;
    return;
  }

  toLevelUp = pToLevelUp;
  return;
}

int Player::getGoldCount( void ) const {
  return goldCount;
}

int Player::getLevel( void ) const {
  return level;
}

int Player::getExperience( void ) const {
  return experience;
}

int Player::getToLevelUp( void ) const {
  return toLevelUp;
}

void Player::information( void ) {
  std::cout << std::endl;
  
  std::cout << "  Level: " << getLevel() << std::endl;
  std::cout << "  Current Health: " << getHealth() << std::endl;
  std::cout << "  Max Health: " <<  getMaxHealth() << std::endl;
  std::cout << "  Normal Damage: " << getDamage() << std::endl;
  std::cout << "  Armor: " << getArmor() << std::endl;
  std::cout << "  Crit Chance: " << getCritChance() << std::endl;
  std::cout << "  Gold Count: " << getGoldCount() << std::endl;
  std::cout << "  Experience: " << getExperience() << std::endl;  
  std::cout << "  Needed Experience To Level Up: " << getToLevelUp()  << std::endl;
  
  std::cout << std::endl;

  return;
}

void Player::drinkPotion( void ) {  
  if( 0 == getPotionCount() ) {
    std::cout << "There is no potion chance for player" << std::endl;
  }
  else {
    std::cout << "Player drunk potion" << std::endl;
    setPotionCount( getPotionCount() - 1 );

    int health = ( int ) getHealth() + CHARACTER_POTION_VALUE;

    health = health % ( CHARACTER_MAX_HEALTH_VALUE + 1 );
    setHealth( ( double ) health );
  }

  return;
}

void Player::addExperience( const int pExperience ) {
  std::cout << "Experience " << pExperience << " is added to player" << std::endl;
  return;
}

void Player::levelUp( void ) {
  std::cout << "Player level up" << std::endl;
  return;
}
  


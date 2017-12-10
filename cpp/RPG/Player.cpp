#include "Player.h"

void Player::setXCoordinate( const int x ) {
  if( x < 0 ) {
    std::cout << "Player x coordinate can not be negative" << std::endl;
    return;
  }

  xCoordinate = x;
  return;
}

void Player::setYCoordinate( const int y ) {
  if( y < 0 ) {
    std::cout << "Player y coordinate can not be negative" << std::endl;
    return;
  }

  yCoordinate = y;
  return;
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

int Player::getXCoordinate( void ) const {
  return xCoordinate;
}

int Player::getYCoordinate( void ) const {
  return yCoordinate;
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

double Player::attack( void ) {
  double att = 1.0;

  std::cout << "Player attacked" << std::endl;

  return att;
}

void Player::information( void ) {
  std::cout << "Player Object" << std::endl;;
  return;
}

void Player::drinkPotion( void ) {
  std::cout << "Player drunk potion" << std::endl;
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
  


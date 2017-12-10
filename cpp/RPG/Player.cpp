#include "Player.h"

Player::Player( void ) : Character() {
  setGoldCount( 0 );
  setExperience( PLAYER_MIN_EXPERIENCE + ( rand() % ( PLAYER_MAX_EXPERIENCE - PLAYER_MIN_EXPERIENCE + 1 ) ) );
  setPotionCount( PLAYER_POTION_CHANCE );
  setLevel( PLAYER_STARTUP_LEVEL );
}

Player::~Player() {

}

void Player::setGoldCount( const int pGoldCount ) {
  if( pGoldCount < 0 ) {
    cout << "Player gold count can not be negative" << endl;
    return;
  }

  goldCount = pGoldCount;
  return;
}

void Player::setLevel( const int pLevel ) {
  if( pLevel < PLAYER_STARTUP_LEVEL ) {
    cout << "Player level has to be " << PLAYER_STARTUP_LEVEL << " at least" << endl;
    return;
  }

  level = pLevel;
  return;
}

void Player::setExperience( const int pExperience ) {
  if( pExperience < 0 ) {
    cout << "Player experience can not be negative" << endl;
    return;
  }

  experience = pExperience;
  return;
}

void Player::setToLevelUp( const int pToLevelUp ) {
  if( pToLevelUp < 0 ) {
    cout << "Player to level up can not be negative" << endl;
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
  cout << endl;
  
  cout << "  Level: " << getLevel() << endl;
  cout << "  Current Health: " << getHealth() << endl;
  cout << "  Max Health: " <<  getMaxHealth() << endl;
  cout << "  Normal Damage: " << getDamage() << endl;
  cout << "  Armor: " << getArmor() << endl;
  cout << "  Crit Chance: " << getCritChance() << endl;
  cout << "  Gold Count: " << getGoldCount() << endl;
  cout << "  Experience: " << getExperience() << endl;  
  cout << "  Needed Experience To Level Up: " << getToLevelUp()  << endl;
  
  cout << endl;

  return;
}

void Player::drinkPotion( void ) {  
  if( 0 == getPotionCount() ) {
    cout << "There is no potion chance for player" << endl;
  }
  else {
    cout << "Player drunk potion" << endl;
    setPotionCount( getPotionCount() - 1 );

    double health = getHealth() + ( double ) CHARACTER_POTION_VALUE;

    if( health > CHARACTER_MAX_HEALTH_VALUE ) {
      health = ( double ) CHARACTER_MAX_HEALTH_VALUE;
    }
    
    setHealth( health );
  }

  return;
}

void Player::addExperience( const int pExperience ) {
  cout << "Experience " << pExperience << " is added to player" << endl;
  return;
}

void Player::levelUp( void ) {
  cout << "Player level up" << endl;
  return;
}
  


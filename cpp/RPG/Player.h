#ifndef PLAYER_H
#define PLAYER_H

#include "Character.h"

// potion chance for player
#define PLAYER_POTION_CHANCE ( 5 )

// player minimum experience
#define PLAYER_MIN_EXPERIENCE ( 20 )

// player maximum experience
#define PLAYER_MAX_EXPERIENCE ( 80 )

// player startup level
#define PLAYER_STARTUP_LEVEL ( 1 )

// Player class
class Player : public Character {
 public:
  Player( void );
  virtual ~Player();
  
  void setGoldCount( const int pGoldCount );
  void setLevel( const int pLevel );
  void setExperience( const int pExperience );
  void setToLevelUp( const int pToLevelUp );

  int getGoldCount( void ) const;
  int getLevel( void ) const;
  int getExperience( void ) const;
  int getToLevelUp( void ) const;

  void information( void );
  void drinkPotion( void );
  void addExperience( const int pExperience );
  void levelUp( void );
  
 private:
  int goldCount;
  int level;
  int experience;
  int toLevelUp;
};

#endif

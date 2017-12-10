#ifndef PLAYER_H
#define PLAYER_H

#include "Character.h"

#define PLAYER_POTION_CHANCE ( 5 )

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

#ifndef PLAYER_H
#define PLAYER_H

#include "Character.h"

class Player : public Character {
 public:
  void setXCoordinate( const int x );
  void setYCoordinate( const int y );
  void setGoldCount( const int pGoldCount );
  void setLevel( const int pLevel );
  void setExperience( const int pExperience );
  void setToLevelUp( const int pToLevelUp );

  int getXCoordinate( void ) const;
  int getYCoordinate( void ) const;
  int getGoldCount( void ) const;
  int getLevel( void ) const;
  int getExperience( void ) const;
  int getToLevelUp( void ) const;

  double attack( void );
  void information( void );
  void drinkPotion( void );
  void addExperience( const int pExperience );
  void levelUp( void );
  
 private:
  int xCoordinate;
  int yCoordinate;
  int goldCount;
  int level;
  int experience;
  int toLevelUp;
};

#endif

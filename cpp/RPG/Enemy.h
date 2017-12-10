#ifndef ENEMY_H
#define ENEMY_H

#include "Character.h"

class Enemy : public Character {
 public:
  void setExperienceAmount( const int eExperienceAmount );
  void setPotionChance( const double ePotionChance );

  int getExperienceAmount( void ) const;
  double getPotionChance( void ) const;

  void information( void );
  void drinkPotion( void );
  void decideAction( void );
  
 private:
  int experienceAmount;
  double potionChance;

};

#endif

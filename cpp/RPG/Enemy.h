#ifndef ENEMY_H
#define ENEMY_H

#include "Character.h"

#define ENEMY_POTION_CHANCE ( 1 )

#define ENEMY_POTION_VALUE ( 500 )

#define ENEMY_MAX_HEALTH_VALUE ( 1000 )

// Enemy class
class Enemy : public Character {
 public:
  Enemy();
  virtual ~Enemy();
  
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

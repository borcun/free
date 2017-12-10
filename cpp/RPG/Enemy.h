#ifndef ENEMY_H
#define ENEMY_H

#include "Character.h"

// enemy minimum startup health
#define ENEMY_STARTUP_MIN_HEALTH ( 500 )

// enemy maximum startup health
#define ENEMY_STARTUP_MAX_HEALTH ( 1000 )

// enemy startup armor
#define ENEMY_STARTUP_ARMOR ( 20 )

// enemy startup damage
#define ENEMY_STARTUP_DAMAGE ( 50 )

// enemy startup minimum potion count
#define ENEMY_STARTUP_MIN_POTION_COUNT ( 1 )

// enemy startup maximum potion count
#define ENEMY_STARTUP_MAX_POTION_COUNT ( 2 )

// enemy potion value
#define ENEMY_POTION_VALUE ( 500 )

// enemy start up minimum xp
#define ENEMY_STARTUP_MIN_XP ( 300 )

// enemy start up maximum xp
#define ENEMY_STARTUP_MAX_XP ( 500 )

// enemy minimum critical hit chance ( %4.0 )
#define ENEMY_MIN_CRITICAL_HIT_CHANCE ( 40 )

// enemy maximum critical hit chance ( %7.5 )
#define ENEMY_MAX_CRITICAL_HIT_CHANCE ( 75 )

// enemy potion chance ratio by health ( %20 )
#define ENEMY_POTION_CHANCE_BY_HEALT ( 20 )

// enemy potion chance to drink ratio ( %80 )
#define ENEMY_POTION_CHANCE_TO_DRINK_RATIO ( 80 )

// Enemy class
class Enemy : public Character {
 public:
  Enemy();
  virtual ~Enemy();
  
  void setExperienceAmount( const int eExperienceAmount );
  int getExperienceAmount( void ) const;

  void information( void );
  void drinkPotion( void );
  void decideAction( void );
  
 private:
  int experienceAmount;

};

#endif

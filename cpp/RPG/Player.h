#ifndef PLAYER_H
#define PLAYER_H

#include "Character.h"

// potion chance for player
#define PLAYER_POTION_COUNT ( 5 )

// player level up experience
#define PLAYER_LEVEL_UP_XP ( 2000 )

// player startup level
#define PLAYER_STARTUP_LEVEL ( 1 )

// player startup experience
#define PLAYER_STARTUP_XP ( 0 )

// player startup health
#define PLAYER_STARTUP_HEALTH ( 1000 )

// player startup damage
#define PLAYER_STARTUP_DAMAGE ( 100 )

// player startup armor
#define PLAYER_STARTUP_ARMOR ( 20 )

// player critical hit chance ( %8.5 )
#define PLAYER_CRITICAL_HIT_CHANCE ( 85 )

// player startup gold count
#define PLAYER_STARTUP_GOLD_COUNT ( 2000 )

// player health increment after fight
#define PLAYER_HEALTH_INC_AFTER_FIGHT ( 300 )

// player armor increment after fight
#define PLAYER_ARMOR_INC_AFTER_FIGHT ( 3 )

// player critical chance increment after fight ( %0.5 )
#define PLAYER_CRIT_CHANCE_INC_AFTER_FIGHT ( 5 )

// player damage increment after fight
#define PLAYER_DAMAGE_INC_AFTER_FIGHT ( 35 )

// player level up increment after fight
#define PLAYER_LEVEL_UP_INC_AFTER_FIGHT ( 1000 )

// player potion count increment ratio ( %30 )
#define PLAYER_POTION_COUNT_INC_RATIO ( 30 )

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

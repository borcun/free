#ifndef CHARACTER_H
#define CHARACTER_H

#include "GameObject.h"
#include <stdlib.h>

#define CHARACTER_ATTACK_VALUE ( 100.0 )

#define CHARACTER_ARMOR_VALUE ( 1000 )

#define CHARACTER_POTION_VALUE ( 500 )

#define CHARACTER_MAX_HEALTH_VALUE ( 1000 )

// Character class
class Character : public GameObject {
 public:
  Character();
  virtual ~Character();
  
  void setHealth( const double cHealth );
  void setMaxHealth( const double cMaxHealth );
  void setArmor( const double cArmor );
  void setDamage( const double cDamage );
  void setCritChance( const double cCritChance );
  void setPotionCount( const int cPotionCount );

  double getHealth( void ) const;
  double getMaxHealth( void ) const;
  double getArmor( void ) const;
  double getDamage( void ) const;
  double getCritChance( void ) const;
  int getPotionCount( void ) const;
  
  bool alive( void ) const;

  double attack( void );
  void takeDamage( const double damageAmount );

  virtual void drinkPotion( void ) = 0;
  
 private:
  double health;
  double maxHealth;
  double armor;
  double damage;
  double critChance;
  int potionCount;
  bool isAlive;
};

#endif

#ifndef CHARACTER_H
#define CHARACTER_H

#include "GameObject.h"
#include <iostream>

class Character : public GameObject {
 public:
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

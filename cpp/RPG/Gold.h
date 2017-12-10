#ifndef GOLD_H
#define GOLD_H

#include "GameObject.h"
#include <iostream>

class Gold : public GameObject {
 public:
  void setGoldAmount( const int gold );
  int getGoldAmount( void ) const;
  void information( void );
  
 private:
  int goldAmount;

};

#endif

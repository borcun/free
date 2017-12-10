#ifndef GOLD_H
#define GOLD_H

#include "GameObject.h"
#include <iostream>

#define INV_GOLD_AMOUNT ( -1 )

class Gold : public GameObject {
 public:
  Gold();
  virtual ~Gold();
  
  void setGoldAmount( const int gold );
  int getGoldAmount( void ) const;
  void information( void );
  
 private:
  int goldAmount;

};

#endif

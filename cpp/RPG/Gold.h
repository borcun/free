#ifndef GOLD_H
#define GOLD_H

#include "GameObject.h"
#include <iostream>

// minimum gold count
#define MIN_GOLD_COUNT ( 300 )

// maximum gold count
#define MAX_GOLD_COUNT ( 500 )

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

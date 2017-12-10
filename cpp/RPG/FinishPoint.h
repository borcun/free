#ifndef FINISH_POINT_H
#define FINISH_POINT_H

#include "GameObject.h"
#include <iostream>

class FinishPoint : public GameObject {
 public:
  void setXCoordinate( const int x );
  void setYCoordinate( const int y );
  int getXCoordinate( void ) const;
  int getYCoordinate( void ) const;
  void information( void );

 private:
  int xCoordinate;
  int yCoordinate;

};

#endif

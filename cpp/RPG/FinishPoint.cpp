#include "FinishPoint.h"

void FinishPoint::setXCoordinate( const int x ) {
  if( x < 0 ) {
    std::cout << "The x coordinate can not be negative" << std::endl;
    return;
  }

  xCoordinate = x;
  return;
}

void FinishPoint::setYCoordinate( const int y ) {
  if( y < 0 ) {
    std::cout << "The y coordinate can not be negative" << std::endl;
    return;
  }

  yCoordinate = y;
  return;
}

int FinishPoint::getXCoordinate( void ) const {
  return xCoordinate;
}

int FinishPoint::getYCoordinate( void ) const {
  return yCoordinate;
}

void FinishPoint::information( void ) {
  std::cout << "Finish Point Object" << std::endl;
  return;
}

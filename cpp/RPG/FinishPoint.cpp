#include "FinishPoint.h"

FinishPoint::FinishPoint() : GameObject() {

}

FinishPoint::~FinishPoint() {

}

void FinishPoint::information( void ) {
  cout << "Finish Point Object" << endl;
  cout << "X: " << getXCoordinate() << " Y: " << getYCoordinate() << endl;
  return;
}

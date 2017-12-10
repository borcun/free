#include "GameObject.h"

GameObject::GameObject() {
  xCoordinate = INV_X_COORDINATE;
  yCoordinate = INV_Y_COORDINATE;
  sign = INV_SIGN;
}

GameObject::~GameObject() {

}

void GameObject::setSign( const char cSign ) {
  sign = cSign;
  return;
}
  
void GameObject::setXCoordinate( const int x ) {
  if( x < 0 ) {
    cout << "The x coordinate can not be negative" << endl;
    return;
  }

  xCoordinate = x;
  return;
}

void GameObject::setYCoordinate( const int y ) {
  if( y < 0 ) {
    cout << "The y coordinate can not be negative" << endl;
    return;
  }

  yCoordinate = y;
  return;
}

int GameObject::getXCoordinate( void ) {
  return xCoordinate;
}

int GameObject::getYCoordinate( void ) {
  return yCoordinate;
}

char GameObject::getSign( void ) {
  return sign;
}

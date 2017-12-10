#include "GameObject.h"

GameObject::~GameObject() {

}

void GameObject::setSign( const char cSign ) {
  sign = cSign;
  return;
}
  
void GameObject::setXCoordinate( const int x ) {
  if( x < 0 ) {
    std::cout << "The x coordinate can not be negative" << std::endl;
    return;
  }

  xCoordinate = x;
  return;
}

void GameObject::setYCoordinate( const int y ) {
  if( y < 0 ) {
    std::cout << "The y coordinate can not be negative" << std::endl;
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

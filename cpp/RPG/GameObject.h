#ifndef GAME_OBJECT_H
#define GAME_OBJECT_H

#include <iostream>

#define INV_X_COORDINATE ( -1 )
#define INV_Y_COORDINATE ( -1 )
#define INV_SIGN 'X'

using namespace std;

class GameObject {
 public:
  GameObject( void );
  virtual ~GameObject();
  
  void setSign( const char cSign );
  void setXCoordinate( const int x );
  void setYCoordinate( const int y );
  
  int getXCoordinate( void );
  int getYCoordinate( void );
  char getSign( void ); 

  virtual void information( void ) = 0;
  
 private:
  int xCoordinate;
  int yCoordinate;
  char sign;

};

#endif

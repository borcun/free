#ifndef GAME_OBJECT_UTIL_H
#define GAME_OBJECT_UTIL_H

#include <iostream>
#include <cstdlib>

using namespace std;

// invalid x coordinate
#define INV_X_COORDINATE ( -1 )

// invalid y coordinate
#define INV_Y_COORDINATE ( -1 )

// invalid sign
#define INV_SIGN 'X'

// function that returns random number between min and max
inline int randNum( const int min, const int max ) {
  return min + ( rand() % ( max - min + 1 ) );
}

#endif

#include "GameObjectMap.h"

GameMap *GameMap::gMap = NULL;

GameObjectMap::GameObjectMap( void ) {
  
}

GameMap *GameMap::getMap( void ) {
  if( NULL == gMap ) {
    gMap = new GameMap();
  }

  return gMap;
}

void GameMap::printMap( void ) {
#ifdef __win__
  system( "cls" );
#elif __win__
  system( "clear" );
#endif

  // print map
  
  return;
}

#ifndef GAME_MAP_H
#define GAME_MAP_H

#include "FinishPoint.h"
#include "Gold.h"
#include "Player.h"
#include "Enemy.h"

#define OBJECT_COUNT ( 10 )

class GameMap {
 public:
  static GameMap *getMap( void );
  bool generateMap( const char *mapPath );
  void printMap( void );
    
 private:
  static GameMap *gMap;
  GameObject *goList;

  GameMap( void );
  
};

#endif

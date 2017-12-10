#ifndef RPG_H
#define RPG_H

#include "FinishPoint.h"
#include "Gold.h"
#include "Player.h"
#include "Enemy.h"
#include "cstdio"
#include "cstring"
#include <list>

#define MAP_SIZE ( 10 )
#define MAX_GAME_OBJECT_COUNT ( 10 )
#define FINISH_POINT_SIGN ( 'F' )
#define GOLD_SIGN ( 'G' )
#define ENEMY_SIGN ( 'E' )
#define PLAYER_SIGN ( 'P' )

enum MenuItem {
  MOVE_UP = 1,
  MOVE_DOWN = 2,
  MOVE_RIGHT = 3,
  MOVE_LEFT = 4,
  PLAYER_INFO = 5,
  EXIT = 6
};

class RPG {
 public:
  virtual ~RPG();
  
  static RPG *instance( void );
  
  bool loadMap( const char *mapPath );

  void run( void );
  
 private:
  static RPG *rpg;
  GameObject *gameObject[ MAX_GAME_OBJECT_COUNT ];

  RPG( void );
  
  void printMap( void );
  
};

#endif

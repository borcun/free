#ifndef RPG_H
#define RPG_H

#include "FinishPoint.h"
#include "Gold.h"
#include "Player.h"
#include "Enemy.h"
#include <cstdio>
#include <cstring>
#include <cmath>
#include <cstdlib>
#include <ctime>

#ifdef _WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif

#define MAP_SIZE ( 10 )

#define MAX_GAME_OBJECT_COUNT ( 10 )

#define PLAYER_OBJECT_INDEX ( 0 )

#define MAX_DIST_TO_PLAYER ( 2 )

#define MAX_GOLD_AMOUNT ( 100 )

#define MIN_GOLD_AMOUNT ( 10 )

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
  EXIT_GAME = 6
};


enum AttackType {
  AT_INVALID = 0,
  AT_NORMAL = 1,
  AT_DRINK_POTION = 2
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
  bool isThereEnemy;
  bool isFinishPoint;
  
  RPG( void );

  // function that print map
  void printMap( void );

  // function that prints menu
  int printMenu( void );

  // function that manages fighting between player and enemy
  bool fight( Enemy *enemy );

  // function that clear screen
  void clearScreen( void );

  // function that sleep
  void sleepScreen( void );
  
};

#endif

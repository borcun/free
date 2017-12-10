#ifndef RPG_H
#define RPG_H

#include "RPGUtil.h"
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

class RPG {
 public:
  virtual ~RPG();
  
  static RPG *instance( void );
  bool loadMap( const char *mapPath );
  void run( void );
  
 private:
  static RPG *rpg;
  GameObject *gameObject[ MAX_GAME_OBJECT_COUNT ];
  int choice;
  bool isFinished;
  
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

  // function that checks if target is enemy
  bool isThereEnemy( const int x, const int y );
};

#endif

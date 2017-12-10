#ifndef RPG_UTIL_H
#define RPG_UTIL_H

// map size
#define MAP_SIZE ( 10 )

// maximum game object count
#define MAX_GAME_OBJECT_COUNT ( 10 )

// player game object index in object list
#define PLAYER_OBJECT_INDEX ( 0 )

// maximum distance to player to open symbol
#define MAX_DIST_TO_PLAYER ( 2 )

// maximum gold amount in map
#define MAX_GOLD_AMOUNT ( 100 )

// minimum gold amount in map
#define MIN_GOLD_AMOUNT ( 10 )

// finish point sign
#define FINISH_POINT_SIGN ( 'F' )

// gold sign
#define GOLD_SIGN ( 'G' )

// enemy sign
#define ENEMY_SIGN ( 'E' )

// player sign
#define PLAYER_SIGN ( 'P' )

// player menu option
enum MenuItem {
  MOVE_UP = 1,
  MOVE_DOWN = 2,
  MOVE_RIGHT = 3,
  MOVE_LEFT = 4,
  PLAYER_INFO = 5,
  EXIT_GAME = 6
};

// attack types when player and enemy fight 
enum AttackType {
  AT_INVALID = 0,
  AT_NORMAL = 1,
  AT_DRINK_POTION = 2
};

// potion count after successfull fight
enum FightPotionCount {
  MIN_PC_AFTER_FIGHT = 1,
  MAX_PC_AFTER_FIGHT = 2
};


#endif

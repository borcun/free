#include "RPG.h"

RPG *RPG::rpg = NULL;

RPG::RPG( void ) {
  for( int i=0 ; i < MAX_GAME_OBJECT_COUNT ; ++i ) {
    gameObject[i] = NULL;
  }
}

RPG::~RPG() {
  for( int i=0 ; i < MAX_GAME_OBJECT_COUNT ; ++i ) {
    if( NULL != gameObject[i] ) {
      delete gameObject[i];
    }
  }
}

RPG *RPG::instance( void ) {
  if( NULL == rpg ) {
    rpg = new RPG();
  }

  return rpg;
}

bool RPG::loadMap( const char *mapPath ) {
  if( NULL == mapPath ) {
    std::cout << "RPG map path is NULL" << std::endl;
    return false;
  }
  else if( 0 == strlen( mapPath ) ) {
    std::cout << "RPG map path is invalid" << std::endl;
    return false;
  }
  
  FILE *fPtr = NULL;

  if( NULL == ( fPtr = fopen( mapPath, "r" ) ) ) {
    std::cout << "RPG map file is not opened" << std::endl;
    return false;
  }

  Player *player = new Player();

  player->setSign( PLAYER_SIGN );
  player->setXCoordinate( MAP_SIZE - 1 );
  player->setYCoordinate( 0 );

  gameObject[0] = player;

  int goCount = 1;

  while( !feof( fPtr ) && goCount < MAX_GAME_OBJECT_COUNT ) {
    char entry[ 255 ] = { '\0' };
    char sign;
    int x, y;

    if( NULL != fgets( entry, 255, fPtr ) ) {
      sscanf( entry, "%c%d%d", &sign, &x, &y );
      
      if( x >= MAP_SIZE || y >= MAP_SIZE || x < 0 || y < 0 || ( x == 0 && y == 0 ) ) {
	std::cout << "Invalid Entry : " << sign << " " << x << " " << y << std::endl;
      }
      else {
	switch( sign ) {
	case FINISH_POINT_SIGN: {
	  FinishPoint *finishPoint = new FinishPoint();

	  finishPoint->setSign( FINISH_POINT_SIGN );
	  finishPoint->setXCoordinate( x );
	  finishPoint->setYCoordinate( y );

	  gameObject[ goCount ] = finishPoint;

	  ++goCount;
	    
	  break;
	}
	case GOLD_SIGN: {
	  Gold *gold = new Gold();

	  gold->setSign( GOLD_SIGN );
	  gold->setXCoordinate( x );
	  gold->setYCoordinate( y );

	  gameObject[ goCount ] = gold;
	  
	  ++goCount;

	  break;
	}
	case ENEMY_SIGN: {
	  Enemy *enemy = new Enemy();
	  
	  enemy->setSign( ENEMY_SIGN );
	  enemy->setXCoordinate( x );
	  enemy->setYCoordinate( y );

	  gameObject[ goCount ] = enemy;

	  ++goCount;
	  
	  break;
	}
	default: {
	  std::cout << "Invalid Entry : " << sign << " " << x << " " << y << std::endl;
	  break;
	}
	} // end of switch
      }
    }
  }
  
  fclose( fPtr );

  return true;
}

void RPG::printMap( void ) {
#ifdef __win__
  system( "cls" );
#elif __win__
  system( "clear" );
#endif

  char goMap[ MAP_SIZE ][ MAP_SIZE ] = { { '#' } };

  for( int i=0 ; i < MAP_SIZE ; ++i ) {      
    for( int j=0 ; j < MAP_SIZE ; ++j ) {
      goMap[i][j] = '#';
    }
  }
  
  for( int i = 0 ; i < MAX_GAME_OBJECT_COUNT && NULL != gameObject[i] ; ++i ) {
    int x = gameObject[i]->getXCoordinate();
    int y = gameObject[i]->getYCoordinate();
    char sign = gameObject[i]->getSign();
    
    switch( sign ) {
    case PLAYER_SIGN: {
      for( int j=x-1 ; j <= x+1 ; ++j ) {
	for( int l=y-1 ; l <= y+1 ; ++l ) {
	  if( !( j < 0 || j == MAP_SIZE || l < 0 || l == MAP_SIZE ) ) {
	    goMap[ j % MAP_SIZE ][ l % MAP_SIZE ] = '-';
	  }
	}
      }
      
      goMap[ x % MAP_SIZE ][ y % MAP_SIZE ] = sign;      

      break;
    }
    case ENEMY_SIGN:
    case FINISH_POINT_SIGN:
    case GOLD_SIGN: {
      goMap[ x % MAP_SIZE ][ y % MAP_SIZE ] = sign;
      break;
    }
    } // end of switch
  } // end of for

  std::cout << std::endl;
  
  for( int i=0 ; i < MAP_SIZE ; ++i ) {
    std::cout << " ";
      
    for( int j=0 ; j < MAP_SIZE ; ++j ) {
      std::cout << goMap[i][j] << " ";
    }

    std::cout << std::endl;
  }

  std::cout << std::endl;
  
  return;
}

void RPG::printMenu( void ) {
  int choice = 0;
  bool validOp = false;
  

  std::cout << " [1] Move up" << std::endl;
  std::cout << " [2] Move down" << std::endl;
  std::cout << " [3] Move right" << std::endl;
  std::cout << " [4] Move left" << std::endl;
  std::cout << " [5] Player Info" << std::endl;
  std::cout << " [6] Exit" << std::endl;
  std::cout << " -> " << std::endl;
}

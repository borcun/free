#include "RPG.h"

RPG *RPG::rpg = NULL;

RPG::RPG( void ) {
  srand( time( NULL ) );
  
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
  player->setXCoordinate( 0 );
  player->setYCoordinate( MAP_SIZE - 1 );

  gameObject[ PLAYER_OBJECT_INDEX ] = player;

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
	  gold->setGoldAmount( MIN_GOLD_AMOUNT + ( rand() % ( MAX_GOLD_AMOUNT - MIN_GOLD_AMOUNT + 1 ) ) );

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

    sleepScreen();
    
  } // end of while
  
  fclose( fPtr );

  return true;
}

void RPG::run( void ) {
  int choice = 0;
  bool isClearScreen = true;
    
  isThereEnemy = false;
  isFinishPoint = false;
  
  clearScreen();
  printMap();
      
  while( choice != EXIT_GAME ) {
    if( isClearScreen ) {
      clearScreen();
      printMap();
    }
    
    choice = printMenu();
    
    switch( choice ) {
    case MOVE_UP: {
      if( !isThereEnemy ) {
	int y = gameObject[ PLAYER_OBJECT_INDEX ]->getYCoordinate() - 1;

	if( y < 0 ) {
	  y = MAP_SIZE - 1;
	}
      
	gameObject[ PLAYER_OBJECT_INDEX ]->setYCoordinate( y % MAP_SIZE );
	clearScreen();
	printMap();
      }
      else {
	isClearScreen = true;
      }
      
      break;
    }
    case MOVE_DOWN: {
      if( !isThereEnemy ) {      
	int y = gameObject[ PLAYER_OBJECT_INDEX ]->getYCoordinate() + 1;

	if( y >= MAP_SIZE ) {
	  y = 0;
	}
      
	gameObject[ PLAYER_OBJECT_INDEX ]->setYCoordinate( y % MAP_SIZE );
	clearScreen();
	printMap();
      }
      else {
	isClearScreen = true;
      }
      
      break;
    }
    case MOVE_RIGHT: {
      if( !isThereEnemy ) {
	int x = gameObject[ PLAYER_OBJECT_INDEX ]->getXCoordinate() + 1;
	
	if( x >= MAP_SIZE ) {
	  x = 0;
	}
	
	gameObject[ PLAYER_OBJECT_INDEX ]->setXCoordinate( x % MAP_SIZE );
	clearScreen();
	printMap();
      }
      else {
	isClearScreen = true;
      }

      break;
    }
    case MOVE_LEFT: {
      if( !isThereEnemy ) {
	int x = gameObject[ PLAYER_OBJECT_INDEX ]->getXCoordinate() - 1;

	if( x < 0 ) {
	  x = MAP_SIZE - 1;
	}
	
	gameObject[ PLAYER_OBJECT_INDEX ]->setXCoordinate( x % MAP_SIZE );
	clearScreen();
	printMap();
      }
      else {
	isClearScreen = true;
      }

      break;
    }
    case PLAYER_INFO: {
      if( isThereEnemy ) {
	clearScreen();
	printMap();
      }

      gameObject[ PLAYER_OBJECT_INDEX ]->information();

      isClearScreen = false;
      
      break;
    }
    case EXIT_GAME: {
      break;
    }
    default: {
      std::cerr << std::endl << "  ! Invalid choice is entered" << std::endl;

      sleepScreen();
      clearScreen();
    
      break;
    }
    } // end of switch
  } // end of while

  return;
}

void RPG::printMap( void ) {
  char goMap[ MAP_SIZE ][ MAP_SIZE ] = { { '#' } };

  for( int i=0 ; i < MAP_SIZE ; ++i ) {      
    for( int j=0 ; j < MAP_SIZE ; ++j ) {
      goMap[i][j] = '#';
    }
  }

  int pXCoor = gameObject[ PLAYER_OBJECT_INDEX ]->getXCoordinate();
  int pYCoor = gameObject[ PLAYER_OBJECT_INDEX ]->getYCoordinate();
  
  for( int i = 0 ; i < MAX_GAME_OBJECT_COUNT && NULL != gameObject[i] ; ++i ) {
    char sign = gameObject[i]->getSign();
    
    switch( sign ) {
    case PLAYER_SIGN: {
      for( int j = pYCoor - 1 ; j <= pYCoor + 1 ; ++j ) {
	for( int k = pXCoor - 1 ; k <= pXCoor + 1 ; ++k ) {
	  if( !( j < 0 || j == MAP_SIZE || k < 0 || k == MAP_SIZE ) ) {
	    goMap[ j % MAP_SIZE ][ k % MAP_SIZE ] = '-';
	  }
	}
      }
      
      goMap[ pYCoor % MAP_SIZE ][ pXCoor % MAP_SIZE ] = sign;      

      break;
    }
    case ENEMY_SIGN: {
      int x = gameObject[i]->getXCoordinate() % MAP_SIZE;
      int y = gameObject[i]->getYCoordinate() % MAP_SIZE;
	
      if( pXCoor == x && pYCoor == y ) {
	isThereEnemy = !fight( ( Enemy * ) gameObject[i] );
      }
      else if( sqrt( pow( pXCoor - x, 2 ) + pow( pYCoor - y, 2 ) ) < MAX_DIST_TO_PLAYER ) {
	goMap[ y % MAP_SIZE ][ x % MAP_SIZE ] = sign;
      }
      
      break;
    }
    case FINISH_POINT_SIGN: {
      int x = gameObject[i]->getXCoordinate() % MAP_SIZE;
      int y = gameObject[i]->getYCoordinate() % MAP_SIZE;

      if( pXCoor == x && pYCoor == y ) {
	isFinishPoint = true;
      }
      else if( sqrt( pow( pXCoor - x, 2 ) + pow( pYCoor - y, 2 ) ) < MAX_DIST_TO_PLAYER ) {
	goMap[ y % MAP_SIZE ][ x % MAP_SIZE ] = sign;
      }
      
      break;
    }
    case GOLD_SIGN: {
      int x = gameObject[i]->getXCoordinate() % MAP_SIZE;
      int y = gameObject[i]->getYCoordinate() % MAP_SIZE;

      if( pXCoor == x && pYCoor == y ) {
	int goldAmount = ( ( Gold * ) gameObject[i] )->getGoldAmount();
	int playerGoldCount = ( ( Player * ) gameObject[ PLAYER_OBJECT_INDEX ] )->getGoldCount();
	
	( ( Player * ) gameObject[ PLAYER_OBJECT_INDEX ] )->setGoldCount( goldAmount + playerGoldCount );

	delete gameObject[i];
	gameObject[i] = NULL;
      }
      else if( sqrt( pow( pXCoor - x, 2 ) + pow( pYCoor - y, 2 ) ) < MAX_DIST_TO_PLAYER ) {
	goMap[ y % MAP_SIZE ][ x % MAP_SIZE ] = sign;

	gameObject[i]->information();
      }
      
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


int RPG::printMenu() {
  int choice = 0;
  
  std::cout << " [1] Move up" << std::endl;
  std::cout << " [2] Move down" << std::endl;
  std::cout << " [3] Move right" << std::endl;
  std::cout << " [4] Move left" << std::endl;
  std::cout << " [5] Player Info" << std::endl;
  std::cout << " [6] Exit" << std::endl;
  std::cout << "  -> ";
  std::cin >> choice;

  return choice;
}

bool RPG::fight( Enemy *enemy ) {
  char fightOpt = 'n';

  do {
    std::cout << "Do you fight your enemy?" << std::endl;
    std::cout << " -> ";
    std::cin >> fightOpt;

    if( 'y' == fightOpt || 'Y' == fightOpt ) {
      Player *player = ( Player * ) gameObject[ PLAYER_OBJECT_INDEX ];
      int attackType = AT_INVALID;

      do {  
	clearScreen();
	
	std::cout << std::endl;
	std::cout << "Player [H:" << player->getHealth() << "][A:" << player->getArmor() << "][P:" << player->getPotionCount() << "]" << std::endl;
	std::cout << "Enemy  [H:" << enemy->getHealth()  << "][A:" << enemy->getArmor()  << "][P:" << enemy->getPotionCount()  << "]" << std::endl;

	std::cout << "[1] Attack" << std::endl;
	std::cout << "[2] Drink Potion" << std::endl;
	std::cout << "  -> ";
	std::cin >> attackType;

	if( AT_NORMAL == attackType ) {
	  // critical attack
	  if( 0 == ( rand() % 2 ) ) {
	    enemy->takeDamage( player->attack() * 2 );
	  }
	  else {
	    enemy->takeDamage( player->attack() );
	  }

	  if( !enemy->alive() ) {
	    std::cout << std::endl;
	    std::cout << "Enemy died" << std::endl;

	    sleepScreen();
	    
	    break;
	  }
	  
	  sleepScreen();

	  // enemy may drink potion
	  if( enemy->getHealth() < CHARACTER_POTION_VALUE &&  0 == ( rand() % 2 ) ) {
	    enemy->drinkPotion();
	  }
	  else {
	    // critical attack
	    if( 0 == ( rand() % 2 ) ) {
	      player->takeDamage( enemy->attack() * 2 );
	    }
	    else {
	      player->takeDamage( enemy->attack() );
	    }	  
	  }
	  
	  if( !player->alive() ) {
	    std::cout << std::endl;
	    std::cout << "Player died" << std::endl;

	    sleepScreen();
	    
	    break;
	  }
	}
	else if( AT_DRINK_POTION == attackType ) {
	  clearScreen();
	
	  std::cout << std::endl;
	  std::cout << "Player [H:" << player->getHealth() << "][A:" << player->getArmor() << "][P:" << player->getPotionCount() << "]" << std::endl;
	  std::cout << "Enemy  [H:" << enemy->getHealth()  << "][A:" << enemy->getArmor()  << "][P:" << enemy->getPotionCount()  << "]" << std::endl;

	  std::cout << "[1] Attack" << std::endl;
	  std::cout << "[2] Drink Potion" << std::endl;
	  std::cout << "  -> ";
	  std::cin >> attackType;

	  player->drinkPotion();

	  sleepScreen();

	  // enemy may drink potion
	  if( enemy->getHealth() < CHARACTER_POTION_VALUE &&  0 == ( rand() % 2 ) ) {
	    enemy->drinkPotion();
	  }
	  else {
	    // critical attack
	    if( 0 == ( rand() % 2 ) ) {
	      player->takeDamage( enemy->attack() * 2 );
	    }
	    else {
	      player->takeDamage( enemy->attack() );
	    }	  
	  }
	  
	  if( !player->alive() ) {
	    std::cout << std::endl;
	    std::cout << "Player died" << std::endl;

	    sleepScreen();
	    
	    break;
	  }
	}
	else {
	  std::cout << "Invalid Attack Type, please press 1 or 2" << std::endl;
	}
      }
      while( true );
    }
    else if( 'n' == fightOpt || 'N' == fightOpt ) {
      return false;
    }
    else {
      std::cout << "Invalid Fight Option, please press Y/N" << std::endl;
    }
  } while( 'n' != fightOpt );
    
  return true;
}

void RPG::clearScreen( void ) {

#ifdef _WIN32
    system( "cls" );
#else
    system( "clear" );
#endif

    return;
}

void RPG::sleepScreen( void ) {

#ifdef _WIN32
    Sleep( 1 );
#else
    sleep( 1 );
#endif
    
    return;
}

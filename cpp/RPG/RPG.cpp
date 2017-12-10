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
    cout << "RPG map path is NULL" << endl;
    return false;
  }
  else if( 0 == strlen( mapPath ) ) {
    cout << "RPG map path is invalid" << endl;
    return false;
  }
  
  FILE *fPtr = NULL;

  if( NULL == ( fPtr = fopen( mapPath, "r" ) ) ) {
    cout << "RPG map file is not opened" << endl;
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
	cout << "Invalid Entry : " << sign << " " << x << " " << y << endl;
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
	  cout << "Invalid Entry : " << sign << " " << x << " " << y << endl;
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
  bool noMap = false;
    
  isFinished = false;
  choice = 0;
      
  while( choice != EXIT_GAME || isFinished ) {
    if( !noMap ) {
      clearScreen();
      printMap();
    }
    
    choice = printMenu();

    int x = gameObject[ PLAYER_OBJECT_INDEX ]->getXCoordinate();
    int y = gameObject[ PLAYER_OBJECT_INDEX ]->getYCoordinate();
    
    switch( choice ) {
    case MOVE_UP: {
      noMap = false;
      
      if( !isThereEnemy( x, y - 1 ) ) {
	--y;

	if( y < 0 ) {
	  y = MAP_SIZE - 1;
	}
      
	gameObject[ PLAYER_OBJECT_INDEX ]->setYCoordinate( y % MAP_SIZE );
	clearScreen();
	printMap();

	if( isFinished ) {
	  return;
	}
      }
      
      break;
    }
    case MOVE_DOWN: {
      noMap = false;
      
      if( !isThereEnemy( x, y + 1 ) ) {
	++y;

	if( y >= MAP_SIZE ) {
	  y = 0;
	}
      
	gameObject[ PLAYER_OBJECT_INDEX ]->setYCoordinate( y % MAP_SIZE );
	clearScreen();
	printMap();

	if( isFinished ) {
	  return;
	}
      }
      
      break;
    }
    case MOVE_RIGHT: {
      noMap = false;
      
      if( !isThereEnemy( x + 1, y ) ) {
	++x;
	
	if( x >= MAP_SIZE ) {
	  x = 0;
	}
	
	gameObject[ PLAYER_OBJECT_INDEX ]->setXCoordinate( x % MAP_SIZE );
	clearScreen();
	printMap();

	if( isFinished ) {
	  return;
	}
      }

      break;
    }
    case MOVE_LEFT: {
      noMap = false;
      
      if( !isThereEnemy( x - 1, y ) ) {
	--x;

	if( x < 0 ) {
	  x = MAP_SIZE - 1;
	}
	
	gameObject[ PLAYER_OBJECT_INDEX ]->setXCoordinate( x % MAP_SIZE );
	clearScreen();
	printMap();

      	if( isFinished ) {
	  return;
	}
      }

      break;
    }
    case PLAYER_INFO: {
      gameObject[ PLAYER_OBJECT_INDEX ]->information();
      noMap = true;
      
      break;
    }
    case EXIT_GAME: {
      break;
    }
    default: {
      cerr << endl << "  ! Invalid choice is entered" << endl;

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
	
      if( sqrt( pow( pXCoor - x, 2 ) + pow( pYCoor - y, 2 ) ) < MAX_DIST_TO_PLAYER ) {
	goMap[ y % MAP_SIZE ][ x % MAP_SIZE ] = sign;
      }
      
      break;
    }
    case FINISH_POINT_SIGN: {
      int x = gameObject[i]->getXCoordinate() % MAP_SIZE;
      int y = gameObject[i]->getYCoordinate() % MAP_SIZE;

      if( pXCoor == x && pYCoor == y ) {
	goMap[ y % MAP_SIZE ][ x % MAP_SIZE ] = 'O';
	isFinished = true;
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

  cout << endl;
  
  for( int i=0 ; i < MAP_SIZE ; ++i ) {
    cout << " ";
      
    for( int j=0 ; j < MAP_SIZE ; ++j ) {
      cout << goMap[i][j] << " ";
    }

    cout << endl;
  }

  cout << endl;
  
  return;
}


int RPG::printMenu() {
  int choice = 0;
  
  cout << " [1] Move up" << endl;
  cout << " [2] Move down" << endl;
  cout << " [3] Move right" << endl;
  cout << " [4] Move left" << endl;
  cout << " [5] Player Info" << endl;
  cout << " [6] Exit" << endl;
  cout << "  -> ";
  cin >> choice;

  return choice;
}

bool RPG::fight( Enemy *enemy ) {
  char fightOpt = 'n';

  do {
    cout << endl;
    cout << " Do you fight your enemy?" << endl;
    cout << "  -> ";
    cin >> fightOpt;

    if( 'y' == fightOpt || 'Y' == fightOpt ) {
      Player *player = ( Player * ) gameObject[ PLAYER_OBJECT_INDEX ];
      int attackType = AT_INVALID;

      do {  
	clearScreen();
	
	cout << endl;

	printf( "Player [H:%04d][A:%04d][P:%02d][XP:%03d][L:%03d]\n",
		( int ) player->getHealth(),
		( int ) player->getArmor(),
		player->getPotionCount(),
		player->getExperience(),
		player->getLevel() );
	
	printf( "Enemy  [H:%04d][A:%04d][P:%02d][XP:%03d]\n",
		( int ) enemy->getHealth(),
		( int ) enemy->getArmor(),
		enemy->getPotionCount(),
		enemy->getExperienceAmount() );

	cout << "[1] Attack" << endl;
	cout << "[2] Drink Potion" << endl;
	cout << "  -> ";
	cin >> attackType;

	if( AT_NORMAL == attackType ) {
	  // critical attack
	  if( 0 == randNum( 1, 2 ) ) {
	    enemy->takeDamage( player->attack() * 2 );
	    cout << endl << "Player attacked critically" << endl;
	  }
	  else {
	    enemy->takeDamage( player->attack() );
	    cout << endl << "Player attacked" << endl;
	  }
	  
	  if( !enemy->alive() ) {
	    cout << "Enemy died" << endl;

	    int prevXP = player->getExperience() / player->getToLevelUp();
	    int curXP = ( player->getExperience() + enemy->getExperienceAmount() ) / player->getToLevelUp();

	    // player level up
	    if( curXP > prevXP ) {
	      player->levelUp();
	    }

	    // increase experience after fight
	    player->addExperience( enemy->getExperienceAmount() );
	    
	    sleepScreen();

	    fightOpt = 'n';
	    
	    break;
	  }
	  
	  sleepScreen();

	  // enemy may drink potion
	  if( 0 != enemy->getPotionCount() &&
	      enemy->getMaxHealth() / enemy->getHealth() > ENEMY_POTION_CHANCE_BY_HEALTH &&
	      ENEMY_POTION_CHANCE_TO_DRINK_RATIO > randNum( 0, 100 ) )
	  {
	    enemy->drinkPotion();
	  }
	  else {
	    int enemyCritChance = randNum( 0, 1000 );
	    
	    // critical attack
	    if( ENEMY_MIN_CRITICAL_HIT_CHANCE <= enemyCritChance &&
		enemyCritChance <= ENEMY_MAX_CRITICAL_HIT_CHANCE )
	    {
	      player->takeDamage( enemy->attack() * 2 );
	      cout << endl << "Enemy attacked critically" << endl;
	    }
	    else {
	      player->takeDamage( enemy->attack() );
	      cout << "Enemy attacked" << endl;
	    }
	  }
	  
	  if( !player->alive() ) {
	    cout << "Player died" << endl;

	    sleepScreen();
	    fightOpt = 'n';
	    isFinished = true;
	    
	    break;
	  }

	  sleepScreen();
	}
	else if( AT_DRINK_POTION == attackType ) {
	  cout << endl;
	  player->drinkPotion();

	  sleepScreen();

	  // enemy may drink potion
	  if( 0 != enemy->getPotionCount() &&
	      enemy->getMaxHealth() / enemy->getHealth() > ENEMY_POTION_CHANCE_BY_HEALTH &&
	      ENEMY_POTION_CHANCE_TO_DRINK_RATIO > randNum( 0, 100 ) )
	  {	    
	    enemy->drinkPotion();
	  }
	}
	else {
	  cout << "Invalid Attack Type, please press 1 or 2" << endl;
	}
      }
      while( true );
    }
    else if( 'n' == fightOpt || 'N' == fightOpt ) {
      return false;
    }
    else {
      cout << "Invalid Fight Option, please press Y/N" << endl;
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

bool RPG::isThereEnemy( const int x, const int y ) {
  for( int i = 0 ; i < MAX_GAME_OBJECT_COUNT ; ++i ) {
    if( NULL != gameObject[i] &&
        gameObject[i]->getXCoordinate() == x &&
	gameObject[i]->getYCoordinate() == y &&
	gameObject[i]->getSign() == ENEMY_SIGN )
      {
	if( fight( ( Enemy * ) gameObject[i] ) ) {
	  delete gameObject[i];
	  gameObject[i] = NULL;
	  
	  return false;
	}

	return true;
      }
  }
  
  return false;
}

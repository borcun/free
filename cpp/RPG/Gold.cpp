#include "Gold.h"

Gold::Gold() : GameObject() {
  goldAmount = randNum( MIN_GOLD_COUNT, MAX_GOLD_COUNT );
}

Gold::~Gold() {

}

void Gold::setGoldAmount( const int gold ) {
  if( gold < 0 ) {
    cout << "Gold amount can not be negative" << endl;
    return;
  }
  
  goldAmount = gold;
  return;
}

int Gold::getGoldAmount( void ) const {
  return goldAmount;
}

void Gold::information( void ) {
  printf( "[%d][%d] Gold Amount: %d\n", getXCoordinate()+1, getYCoordinate()+1, goldAmount );
  return;
}

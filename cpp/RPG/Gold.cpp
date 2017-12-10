#include "Gold.h"

Gold::Gold() : GameObject() {
  goldAmount = INV_GOLD_AMOUNT;
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
  cout << "Gold Object" << endl;
  cout << "Gold Amount: " << goldAmount << endl;
  return;
}

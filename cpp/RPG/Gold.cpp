#include "Gold.h"

Gold::Gold() : GameObject() {
  goldAmount = INV_GOLD_AMOUNT;
}

Gold::~Gold() {

}

void Gold::setGoldAmount( const int gold ) {
  if( gold < 0 ) {
    std::cout << "Gold amount can not be negative" << std::endl;
    return;
  }
  
  goldAmount = gold;
  return;
}

int Gold::getGoldAmount( void ) const {
  return goldAmount;
}

void Gold::information( void ) {
  std::cout << "Gold Amount: " << goldAmount << std::endl;
  return;
}

#include "Gold.h"

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
  std::cout << "Gold Amount Object" << std::endl;
  return;
}

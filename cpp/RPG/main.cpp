#include "FinishPoint.h"
#include "Gold.h"
#include "Player.h"
#include "Enemy.h"

int main( int argc, char **argv ) {
  GameMap *gMap = GameMap::getMap();
  
  std::cout << "RPG is started" << std::endl;

  delete gMap;
  
  return 0;
}

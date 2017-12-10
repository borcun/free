#include "RPG.h"

int main( int argc, char **argv ) {
  RPG *rpg = RPG::instance();
  
  std::cout << "RPG is started..." << std::endl;

  if( !rpg->loadMap( "map.txt" ) ) {
    std::cout << "RPG map is not loaded" << std::endl;
    delete rpg;

    return -1;
  }

  rpg->run();
  
  delete rpg;
  
  return 0;
}

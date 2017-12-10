#include "RPG.h"

int main( int argc, char **argv ) {
  srand( time( NULL ) );
  
  RPG *rpg = RPG::instance();
  
  cout << "RPG is started..." << endl;

  if( !rpg->loadMap( "map.txt" ) ) {
    cout << "RPG map is not loaded" << endl;
    delete rpg;

    return -1;
  }

  rpg->run();
  
  delete rpg;

  cout << endl << " Game Over" << endl;
  
  return 0;
}

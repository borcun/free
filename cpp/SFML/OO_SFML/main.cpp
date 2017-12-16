#include "TestWindow.h"

void usage( void ) {
  cout << "./test_window <texture file path>" << endl;
  return;
}

int main( int argc, char **argv ) {
  if( argc != 2 ) {
    usage();
    return -1;
  }
  
  TestWindow window( 600, 400, "Test Window" );

  window.setPosition( 400, 400 );

  if( !window.setTextureFile( argv[1] ) ) {
    cerr << "Texture file is not loaded" << endl;
    return -1;
  }

  window.run();
  
  return 0;
}

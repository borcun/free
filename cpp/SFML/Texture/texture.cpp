#include <iostream>
#include <SFML/Graphics.hpp>
#include <unistd.h>

using namespace std;
using namespace sf;

int main() {
  RenderWindow window( VideoMode( 800, 600 ), "Ball" );
  Texture bTexture;
  Sprite sprite;
  Vector2f incr( 0.4f, 0.4f );
  
  window.setPosition( Vector2i( 100, 100 ) );

  if( !bTexture.loadFromFile( "ball.png" ) ) {
    cerr << "Texture file is not loaded" << endl;
    return -1;
  }

  sprite.setTexture( bTexture );
  
  while( window.isOpen() ) {
    Event event;

    while( window.pollEvent( event ) ) {
      if( Event::Closed == event.type ) {
	window.close();
      }
    }

    window.clear( Color::Black );
    window.draw( sprite );
    window.display();

    sprite.setPosition( sprite.getPosition() + incr );
    usleep( 2000 );
  }
  
  return 0;
}

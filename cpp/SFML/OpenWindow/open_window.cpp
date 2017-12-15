#include <SFML/System.hpp>
#include <SFML/Window.hpp>
#include <SFML/Graphics.hpp>
#include <unistd.h>
#include <iostream>

using namespace std;
using namespace sf;

int main() {
  RenderWindow window;

  window.create( VideoMode( 800, 600 ), "My Window" );
  window.setPosition( Vector2i( 100, 100 ) );
  
  while( window.isOpen() ) {
    Event event;

    while( window.pollEvent( event ) ) {
      if( Event::Closed == event.type ) {
	window.close();
      }

      window.clear( Color::Black );
      window.display();
    }
  }
  
  return 0;
}

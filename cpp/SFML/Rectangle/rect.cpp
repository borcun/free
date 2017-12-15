#include <SFML/Graphics.hpp>

using namespace sf;

int main() {
  RenderWindow window( VideoMode( 800, 600 ), "Rectangle" );
  RectangleShape rect( Vector2f( 100.0f, 100.0f ) );

  window.setPosition( Vector2i( 100, 100 ) );

  rect.setFillColor( Color::Red );
  rect.setPosition( 200, 200 );
    
  while( window.isOpen() ) {
    Event event;

    while( window.pollEvent( event ) ) {
      if( Event::Closed == event.type ) {
	window.close();
      }
    }

    window.clear( Color::Black );
    window.draw( rect );
    window.display();
  }
  
  return 0;
}

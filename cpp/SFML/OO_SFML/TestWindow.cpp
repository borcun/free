#include "TestWindow.h"

TestWindow::TestWindow( const unsigned int width, const unsigned height, const char *title  ) {
  window.create( VideoMode( width, height ), title );
}

TestWindow::~TestWindow() {
  if( window.isOpen() ) {
    window.close();
  }
}

void TestWindow::setPosition( const unsigned int x, const unsigned int y ) {
  window.setPosition( Vector2i( x, y ) );
  return;
}

bool TestWindow::setTextureFile( const char *img_path ) {
  if( NULL == img_path ) {
    return false;
  }
  
  if( texture.loadFromFile( img_path ) ) {
    sprite.setTexture( texture );
    return true;
  }

  return false;
}

void TestWindow::run( void ) {
  bool isClosed = false;
  Vector2f incr( 0.4f, 0.4f );

  while( !isClosed && window.isOpen() ) {
    Event event;

    while( window.pollEvent( event ) ) {
      if( Event::Closed == event.type ) {
	isClosed = true;
      }
    }

    window.clear( Color::Black );
    window.draw( sprite );
    window.display();

    sprite.setPosition( sprite.getPosition() + incr );
    usleep( 2000 );
  }

  return;
}
  

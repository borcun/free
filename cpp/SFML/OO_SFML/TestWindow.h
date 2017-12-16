#ifndef TEST_WINDOW_H
#define TEST_WINDOW_H

#include <iostream>
#include <SFML/Graphics.hpp>
#include <unistd.h>

using namespace std;
using namespace sf;

class TestWindow {
 public:
  TestWindow( const unsigned int width, const unsigned height, const char *title  );
  virtual ~TestWindow();
  void setPosition( const unsigned int x, const unsigned int y );
  bool setTextureFile( const char *img_path );
  void run( void );
  
 private:
  RenderWindow window;
  Texture texture;
  Sprite sprite;

};

#endif

#ifndef GAME_OBJECT_H
#define GAME_OBJECT_H

class GameObject {
 public:
  void setSign( const char cSign );
  char getSign( void ); 
  virtual void information( void ) = 0;
  
 private:
  char sign;

};

#endif

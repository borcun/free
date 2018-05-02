#ifndef SQUARE_H
#define SQUARE_H

class Square {
 public:
  Square( const float edge );
  float area( void );
  
 private:
  float m_edge;
};

#endif

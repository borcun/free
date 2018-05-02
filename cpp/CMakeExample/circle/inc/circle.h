#ifndef CIRCLE_H
#define CIRCLE_H

#define PI ( 3.1415 )

class Circle {
 public:
  Circle( const float radius );
  float area( void );
  
 private:
  float m_radius;
  
};

#endif

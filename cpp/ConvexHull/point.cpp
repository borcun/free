#include "point.h"

Point::Point( void ) {
  m_x = 0.0f;
  m_y = 0.0f;
}

Point::Point( const float x, const float y ) {
  set( x, y );
}
  
Point::~Point() {
}

void Point::set( const Point &p ) {
  set( p.getX(), p.getY() );

  return;
}

void Point::set( const float x, const float y ) {
  m_x = x;
  m_y = y;

  return;
}

float Point::getX( void ) const {
  return m_x;
}

float Point::getY( void ) const {
  return m_y;
}

bool Point::operator==( const Point &p ) {
  return m_x == p.getX() && m_y == p.getY();
}

bool Point::operator!=( const Point &p ) {
  return !(m_x == p.getX() && m_y == p.getY());
}

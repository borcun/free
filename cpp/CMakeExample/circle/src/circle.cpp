#include "circle.h"

Circle::Circle( const float radius ) {
  m_radius = radius;
}

float Circle::area( void ) {
  return 2 * PI * m_radius;
}

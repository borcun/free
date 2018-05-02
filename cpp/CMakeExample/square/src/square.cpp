#include "square.h"

Square::Square( const float edge ) {
  m_edge = edge;
}

float Square::area( void ) {
  return m_edge * m_edge;
}

#include "convex_hull.h"

ConvexHull::ConvexHull( void ) {

}

void ConvexHull::setPoints( std::vector< Point > point_set ) {
  m_point_set = point_set;
  return;
}

void ConvexHull::solve( void ) {
  Point leftmost;
  Point endpoint;
  std::vector< Point > solution;

  for( int i=0; i < m_point_set.size(); ++i ) {
    if( m_point_set[i].getX() > leftmost.getX() ) {
      leftmost.set( m_point_set[i] );
    }
  }

  solution.push_back( leftmost );
  endpoint = leftmost;
  
  do {    
    float angle = 0.0f;
    int index = 0;

    for( int i=0; i < m_point_set.size(); ++i ) {
      float cur_point = 0.0f;
	
      cur_point = ( m_point_set[i].getY() - endpoint.getY() ) /
	( m_point_set[i].getX() - endpoint.getX() );

      cur_point = sin( cur_point * PI / 180.0f );

      if( cur_point > angle ) {
	angle = cur_point;
	index = i;
      }
    }

    endpoint = m_point_set[ index ];
    solution.push_back( endpoint );
    
  } while( leftmost != endpoint ); 

  for( int i=0; i < solution.size(); ++i ) {
    std::cout << (i+1) << ". (" << solution[i].getX() << ", "
	      << solution[i].getY() << ")" << std::endl;
  }

  return;
}

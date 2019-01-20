#include "convex_hull.h"
#include <cstdlib>
#include <ctime>

void usage( void ) {
  std::cout << "./convex_hull <number of points>" << std::endl;
  return;
}

int main( int argc, char **argv ) {
  if( argc != 2 ) {
    usage();
    return -1;
  }
  
  srand( time( NULL ) );
  
  std::vector< Point > points;
  ConvexHull convex_hull;
  
  for( int i=0; i < atoi( argv[1] ); ++i ) {
    Point p( ( float ) ( rand() % 1000 ) / 100.0f,
	     ( float ) ( rand() % 1000 ) / 100.0f );

    points.push_back( p );

    std::cout << (i+1) << ". " << p.getX() << ", " << p.getY() << std::endl;
  }

  std::cout << "-----" << std::endl;

  convex_hull.setPoints( points );
  convex_hull.solve();
  
  return 0;
}

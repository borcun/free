#ifndef CONVEX_HULL_H
#define CONVEX_HULL_H

#include <iostream>
#include <vector>
#include <cmath>
#include "point.h"

#define PI ( 3.1415 )

class ConvexHull {
 public:
  ConvexHull( void );
  void setPoints( std::vector< Point > point_set );
  void solve( void );

 private:
  std::vector< Point > m_point_set;
};

#endif

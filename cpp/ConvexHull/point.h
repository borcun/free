#ifndef POINT_H
#define POINT_H

class Point {
 public:
  Point( void );  
  Point( const float x, const float y );  
  virtual ~Point();

  void set( const Point &p );
  void set( float x, float y );
  float getX( void ) const;
  float getY( void ) const;
  bool operator==( const Point &p );
  bool operator!=( const Point &p );

 private:
  float m_x;
  float m_y;

};

#endif

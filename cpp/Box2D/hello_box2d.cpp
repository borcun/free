#include <iostream>
#include <Box2D/Box2D.h>

int main() {
  b2Vec2 gravity( 0.0f, -10.0f );
  b2World world( gravity );
  b2BodyDef groundDef;
  b2PolygonShape groundShape;
  b2Body *ground;

  std::cout << "Hello Box2d" << std::endl;
  
  groundDef.position.Set( 0.0f, -10.0f );
  groundShape.SetAsBox( 50.0f, 10.0f );
  
  ground = world.CreateBody( &groundDef );
  ground->CreateFixture( &groundShape, 0.0f );

  return 0;
}

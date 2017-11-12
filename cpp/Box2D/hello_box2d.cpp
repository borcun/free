/**
 * @file hello_box2d.cpp
 * @brief Hello World example for Box2D library
 * @date Oct 12, 2017
 * @author boo
 * @copyright free
 */

#include <iostream>
#include <cstdio>
#include <Box2D/Box2D.h>

int main() {
  // gravity defines gravity value of world
  b2Vec2 gravity( 0.0f, -10.0f );
  b2World world( gravity );

  // body definition instances are used to specify configuration for body instances
  b2BodyDef groundDef;
  b2BodyDef bodyDef;

  // shape instances are used to specify fixture to body instances
  b2PolygonShape groundShape;
  b2PolygonShape bodyShape;

  // body references
  b2Body *ground;
  b2Body *body;

  // fixture instance is used to pass more properties to a body instance
  b2FixtureDef fixtureDef;

  // time step defines simulation loop
  float32 timeStep = 1.0f / 60.0f;

  // velocity and position iterators are constraints for simulation
  int32 velIter = 6, posIter = 2;
  
  std::cout << "Hello Box2d" << std::endl;

  // ground and body properties are set
  // if a body is not defined as dynamic, it is static by default
  groundDef.position.Set( 0.0f, -10.0f );
  bodyDef.position.Set( 0.0f, 4.0f );
  bodyDef.type = b2_dynamicBody;
  
  // shape properties, whereas ground is rectangle, body is square
  groundShape.SetAsBox( 50.0f, 10.0f );
  bodyShape.SetAsBox( 1.0f, 1.0f );

  // create ground and set its fixture
  ground = world.CreateBody( &groundDef );
  ground->CreateFixture( &groundShape, 0.0f );

  // fixture properties are set for body reference
  fixtureDef.shape = &bodyShape;
  fixtureDef.density = 1.0f;
  fixtureDef.friction = 0.3f;

  // create body and set its fixture
  body = world.CreateBody( &bodyDef );
  body->CreateFixture( &fixtureDef );

  // start simulation loop
  for( int32 i = 0; i < 60 ; ++i ) {
    // Step function performs an iteration of simulation loop with iteration solver parameters
    world.Step( timeStep, velIter, posIter );

    b2Vec2 pos = body->GetPosition();
    float32 angle = body->GetAngle();

    printf( "x: %4.2f y: %4.2f ang: %4.2f\n", pos.x, pos.y, angle );
  }
  
  return 0;
}

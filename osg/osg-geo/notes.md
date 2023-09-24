**Vertices and vertex attributes**
The Vertex is the atomic element of geometry primitives. It uses several numeric attributes to
describe a point in 2D or 3D spaces, including vertex position, color, normal and texture coordinates,
fog coordinates, and so on. The position value is always required, and other attributes to be specified
per vertex, and can create different arrays in which to store each of them. All attribute arrays are
supported by the osg::Geometry class with the corresponding set*Array() methods.

  * Position -> setVertexArray()
  * Normal -> setNormalArray()
  * Color -> setColorArray()
  * Secondary Color -> setSecondaryColorArray()
  * Fog Coordinate -> setFogCoordArray()
  * Texture Coordinate -> setTexCoordArray()
  * Other General Attributes -> setVertexAttribArray()

OSG provides binding methods to make the work more convenient. For instance, developers may call the public
method setColorBinding() of an osg::Geometry object geom, and take an enumerate as the parameter:

  > geom->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
  > geom->setColorBinding(osg::Geometry::BIND_OVERALL);

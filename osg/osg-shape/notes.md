**How OpenGL draws objects**

OpenGL uses *geometry primitives* to draw different objects. The geometry primitives are points, lines,
triangles or polygonal faces determines how OpenGL sorts and renders its associated vertex data. The
easist way to render a primitive is to specify a list of vertices between the glBegin() and glEnd()
pair, which is called *immediate mode*, but it is inefficient in most cases. (OSG has an internal 
osg::GLBeginEndAdapter class that is used to perform basic shape drawing operations. This class 
enables the use of vertex arrays in the style of a glBegin() and glEnd() pair, which makes the 
implementation of basic shapes easy to understand and extend.)

The vertex data, including vertex coordinates, normals, colors and texture coordinates, can also be
stored in various arrays. Primitives will be formed by dereferencing and indexing the array elements.
This method, named *vertex array*, reduces redundant shared vertices and thus performs better than
*immediate mode*.

*Display lists* also significantly improve application performance, because all vertex and pixel data
are compiled and copied into the graphics memory. The prepared primitives can be reused repeatedly,
without transmitting data over and over again. It helps a lot in drawing static geometries.

The *vertex buffer object (VBO)* mechanism allows *vertex array* data to be stored in high-performance
memory. This provides a more efficient solution for transferring dynamic data.

By default, OSG uses vertex arrays and display lists to manage and render geometries. However, this 
may change depending on different data types and rendering strategies.

We would like to also call attention to the removal of immediate mode and display lists in OpenGL ES 
and OpenGL 3.x, for the purpose of producing a more lightweight interface. Of course OpenGL 3.x and 
further versions will keep these deprecated APIs for backward compatibility. However, they are not 
recommended to be used in new code.

**How OSG draw objects**
Everything that can be rendered is implemented as a class derived from *Drawable*. The Drawable class
contains no drawing primitives, since these are provided by subclasses such as *osg::Geometry*. Notice
that a Drawable is not Node, and therefore it cannot be directly added to a scene graph. Instead, Drawable
are attached to *Geodes*, which are scene graph nodes.

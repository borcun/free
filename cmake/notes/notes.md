BASICS
------
 - configure -> generate -> build
 - tools: cmake, ccmake, ctest, cpack
 

BUILD COMMAND
-------------
  cmake -B <build-tree> -S <source-tree>
  cmake --build <build-tree>

GENERATOR
---------
  cmake -G <generator name> <path-to-source>
  
CACHE PARAMETERS
----------------
  cmake -D <var>[:<type>]=<value> <path-to-source>
	cmake -S <source-tree> -B <build-tree> -D CMAKE_BUILD_TYPE=Release

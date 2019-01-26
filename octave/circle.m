# this script draws a circle whose center is at origin, radius is 1.

x = linspace( -1, 1, 1000 );
y = sqrt( 1 - x.^2 );
plot( x, y, x, -y );

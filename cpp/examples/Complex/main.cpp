#include "Complex.h"

int main() {
	Complex c1( 4, 7 );
	Complex c2( 3, -2 );
	Complex c3, c4;
	
	c1.display();
	c2.display();

	c3 = c1 + c2;
	c4 = c1 - c2;

	c3.display();
	c4.display();

	c4.setReel( 8 );

	c4.display();

	return 0;
}

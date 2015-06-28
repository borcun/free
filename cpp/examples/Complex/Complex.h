#ifndef COMPLEX_H
#define COMPLEX_H

#include <iostream>

class Complex {
 private:
	int m_reel;
	int m_imag;

 public:
	Complex();
	Complex( int reel, int imag );
	~Complex();
	void setImag( int imag );
	void setReel( int reel );
	int getImag();
	int getReel();
	void display();
	Complex operator+( Complex c );
	Complex operator-( Complex c );
};

#endif

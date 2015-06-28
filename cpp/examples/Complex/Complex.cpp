#include "Complex.h"

Complex::Complex() {

}

Complex::Complex( int reel, int imag ) {
	m_reel = reel;
	m_imag = imag;
}

Complex::~Complex() {

}

void Complex::setImag( int imag ) {
	m_imag = imag;
}

void Complex::setReel( int reel ) {
	m_reel = reel;
}

int Complex::getImag() {
	return m_imag;
}

int Complex::getReel() {
	return m_reel;
}

void Complex::display() {
	std::cout << m_reel << " + (" << m_imag << "i)" << std::endl;
}

Complex Complex::operator+( Complex c ) {
	Complex temp;

	temp.setReel( m_reel + c.getReel() );
	temp.setImag( m_imag + c.getImag() );

	return temp;
}


Complex Complex::operator-( Complex c ) {
	Complex temp;

	temp.setReel( m_reel - c.getReel() );
	temp.setImag( m_imag - c.getImag() );

	return temp;
}

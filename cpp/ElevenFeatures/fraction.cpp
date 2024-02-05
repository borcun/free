//
//  fraction.cpp
//  ElevenFeatures
//
//  Created by B. Orçun Özkablan on 30.12.2021.
//

#include "fraction.h"

Fraction::Fraction(void) : Fraction(0, 1)
{
    
}

Fraction::Fraction(const int32_t num) : Fraction(num, 1)
{
    
}

Fraction::Fraction(const int32_t num, const int32_t denom)
{
    m_num = num;
    
    if (0 == denom) {
        m_denom = 1;
    }
    else {
        m_denom = denom;
    }
}

Fraction::Fraction(const Fraction &frac) {
    std::cout << "copy constructor" << std::endl;
}

Fraction::~Fraction() {
    std::cout << "destructor" << std::endl;
}

void Fraction::setNumerator(const int num) {
    m_num = num;
    return;
}

void Fraction::setDenominator(const int denom) {
    if (0 == denom) {
        std::cerr << "Could not set denom to zero" << std::endl;
    }
    else {
        m_denom = denom;
    }
    
    return;
}

int32_t Fraction::getNumerator(void) const {
    return m_num;
}

int32_t Fraction::getDenominator(void) const {
    return m_denom;
}

double Fraction::getResult(void) {
    return static_cast<double>(m_num) / m_denom;
}

void Fraction::printReduced(void) {
    if (0 == m_num) {
        std::cout << m_num << "/" << m_denom << std::endl;
    }
    else {
        int32_t bcd = getBCD(m_num, m_denom);
        
        std::cout << (m_num / bcd) << "/" << (m_denom / bcd) << std::endl;
    }
    
    return;
}

const Fraction &Fraction::operator*(const Fraction &frac) {
    const int32_t num = m_num * frac.m_num;
    const int32_t denom = m_denom * frac.m_denom;
    
    std::cout << __FILE__ << ":" << __LINE__ << std::endl;
    
    return Fraction(num, denom);
}

int32_t Fraction::getBCD(int32_t a, int32_t b) {
    if (0 == a) {
        return b;
    }
    
    return getBCD(b % a, a);
}

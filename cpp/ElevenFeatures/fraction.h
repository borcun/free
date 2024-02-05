//
//  fraction.h
//  ElevenFeatures
//
//  Created by B. Orçun Özkablan on 30.12.2021.
//

#ifndef fraction_h
#define fraction_h

#include <iostream>

class Fraction {
public:
    Fraction(void);
    Fraction(const int32_t num);
    Fraction(const int32_t num, const int32_t denom);
    Fraction(const Fraction &frac);
    virtual ~Fraction();
    void setNumerator(const int32_t num);
    void setDenominator(const int32_t denom);
    int32_t getNumerator(void) const;
    int32_t getDenominator(void) const;
    double getResult(void);
    void printReduced(void);
    
    const Fraction &operator*(const Fraction &frac);
    
private:
    int32_t m_num;
    int32_t m_denom;
    
    int32_t getBCD(int32_t a, int32_t b);
};

#endif /* fraction_h */

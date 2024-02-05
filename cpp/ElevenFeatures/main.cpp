//
//  main.cpp
//  ElevenFeatures
//
//  Created by B. Orçun Özkablan on 29.12.2021.
//

#include "fraction.h"
#include <cstdio>

const Fraction &operator*(const Fraction &frac1, const Fraction &frac2);

void printPtrAddress(int *iptr);
void printRefAddress(int &ref);
Fraction checkCopyCtor(Fraction fraction);
void writeVertical(int32_t num);

class A {
public:
   A(void) {
      std::cout << "default " << __FUNCTION__ << std::endl;
   }
   
   A(int32_t x) : m_x(x) {
      std::cout << __FUNCTION__ << std::endl;
   }
   
private:
   int32_t m_x;
};

class B : public A {
public:
   B(int32_t x) : m_x(x) {
      std::cout << __FUNCTION__ << std::endl;
   }
private:
   int32_t m_x;
};

int main(int argc, const char * argv[]) {
   Fraction frac1;
   Fraction frac2;
   Fraction frac3;
   Fraction frac4;
   int x = 3;
   int *iptr = nullptr;

   frac1.setNumerator(12);
   frac1.setDenominator(18);
   
   frac2.setNumerator(7);
   frac2.setDenominator(11);
   
   frac3 = 2 * frac2;
   frac4 = frac2 * 2;
   
   std::cout << frac1.getResult() << std::endl;
   std::cout << frac2.getResult() << std::endl;
   std::cout << frac3.getResult() << std::endl;
   std::cout << frac4.getResult() << std::endl;

   frac1.printReduced();
   frac2.printReduced();
   frac3.printReduced();
   frac4.printReduced();
   
   iptr = &x;
   
   printf("iptr: 0x%x\n", iptr);
   printf("iptr addr: 0x%x\n", &iptr);
   
   printPtrAddress(&x);
   printRefAddress(x);

   printf("x: %d\n", x);
   
   checkCopyCtor(frac1);
   
   writeVertical(1234);
   
   std::cout << "--------" << std::endl;
   B b(3);
   std::cout << "--------" << std::endl;

   return 0;
}

const Fraction &operator*(const Fraction &frac1, const Fraction &frac2) {
   std::cout << __FILE__ << ":" << __LINE__ << std::endl;
   
   return Fraction(frac1.getNumerator() * frac2.getNumerator(),
                    frac2.getDenominator() * frac2.getDenominator());
}

void printPtrAddress(int *iptr) {
   printf("iptr: 0x%x\n", iptr);
   printf("iptr addr: 0x%x\n", &iptr);
   
   return;
}

void printRefAddress(int &ref) {
   printf("ref: %d\n", ref);
   printf("ref addr: 0x%x\n", &ref);
   
   ref = 10;
   
   return;
}

Fraction checkCopyCtor(Fraction fraction) {
   return fraction;
}

void writeVertical(int32_t num) {
   if (num > 10) {
      writeVertical(num / 10);
   }

   std::cout << (num % 10) << std::endl;   
   return;
}


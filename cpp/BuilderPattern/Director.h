#ifndef DIRECTOR_H
#define DIRECTOR_H

#include "AppleBuilder.hpp"
#include "SamsungBuilder.hpp"

class Director {
public:
   Director(PhoneBuilder *builder) {
      this->builder = builder;
   }
   
   virtual ~Director() = default;
   
   void setBuilder(PhoneBuilder *builder) {
      this->builder = builder;
   }
   
   Phone build() {
      if (nullptr != builder) {
         builder->buildScreen();
         builder->buildCamera();
         builder->buildColor();
      }
      
      return builder->getPhone();
   }
   
private:
   PhoneBuilder *builder = nullptr;
};

#endif

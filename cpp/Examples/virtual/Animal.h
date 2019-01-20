#ifndef ANIMAL_H
#define ANIMAL_H

#include <iostream>

class Animal {
public:
  Animal( void ) {
    std::cout << "Animal Ctor" << std::endl;
    fn();
  }

  virtual void fn( void ) {
    std::cout << "Animal Fn" << std::endl;
    _n = 1;

    return;
  }

  int getn() { return _n; }

protected:
  int _n;
};

class Bee : public Animal
{
public:
  Bee( void ) : Animal() {
    std::cout << "Bee Ctor" << std::endl;
    fn();
  }

  virtual void fn( void ) {
    std::cout << "Bee Fn" << std::endl;
    _n = 2;

    return;
  }
};

#endif

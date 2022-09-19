#ifndef KNIGHT_HPP
#define KNIGHT_HPP

#include "Warrior.h"

class Knight : public Warrior {
public:
  Knight(const std::string &name);
  ~Knight();
  void fight(void);
};

#endif

#ifndef QUEEN_HPP
#define QUEEN_HPP

#include "Warrior.h"

class Queen : public Warrior {
public:
  Queen(const std::string &name);
  ~Queen();
  void fight(void);
};

#endif

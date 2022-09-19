/**
 * @file King that uses bow and arrow
 * @author boo
 * @date November 2, 2013
 */
#ifndef KING_HPP
#define KING_HPP

#include "Warrior.h"

class King : public Warrior {
public:
  King(const std::string &name);
  ~King();
  void fight(void);
};

#endif

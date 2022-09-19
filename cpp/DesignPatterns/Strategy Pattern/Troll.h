/* 
 * File:   Troll.h
 * Author: boo
 *
 * Created on November 2, 2013, 11:36 PM
 */

#ifndef TROLL_HPP
#define	TROLL_HPP

#include "Warrior.h"

class Troll : public Warrior {
public:
  Troll(const std::string &name);
  ~Troll();
  void fight(void);
};

#endif

/* 
 * File:   SwordBehaviour.h
 * Author: boo
 *
 * Created on November 2, 2013, 11:27 PM
 */

#ifndef SWORDBEHAVIOUR_HPP
#define	SWORDBEHAVIOUR_HPP

#include <iostream>
#include "WeaponBehaviour.h"

class SwordBehaviour : public WeaponBehaviour {
public:
  ~SwordBehaviour();
  void useWeapon(void);
};

#endif

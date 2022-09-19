/* 
 * File:   BowAndArrowBehaviour.h
 * Author: boo
 *
 * Created on November 2, 2013, 11:23 PM
 */

#ifndef BOWANDARROWBEHAVIOUR_HPP
#define	BOWANDARROWBEHAVIOUR_HPP

#include "WeaponBehaviour.h"

class BowAndArrowBehaviour : public WeaponBehaviour {
public:
  ~BowAndArrowBehaviour();
  void useWeapon(void);
};

#endif

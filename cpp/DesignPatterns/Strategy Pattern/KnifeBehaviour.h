/** 
 * File:   KnifeBehaviour.h
 * Author: boo
 *
 * Created on November 2, 2013, 11:07 PM
 */

#ifndef KNIFEBEHAVIOUR_HPP
#define	KNIFEBEHAVIOUR_HPP

#include "WeaponBehaviour.h"

class KnifeBehaviour : public WeaponBehaviour {
public:
  ~KnifeBehaviour();
  void useWeapon(void);
};

#endif

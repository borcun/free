/** 
 * File:   AxeBehaviour.h
 * Author: boo
 *
 * Created on November 2, 2013, 11:26 PM
 */

#ifndef AXEBEHAVIOUR_HPP
#define	AXEBEHAVIOUR_HPP

#include "WeaponBehaviour.h"

class AxeBehaviour : public WeaponBehaviour {
public:
  ~AxeBehaviour();
  void useWeapon(void);
};

#endif

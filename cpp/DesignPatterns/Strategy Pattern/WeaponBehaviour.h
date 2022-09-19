/* 
 * File:   WeaponBehaviour.hpp
 * Author: boo
 *
 * Created on November 2, 2013, 11:00 PM
 */

#ifndef WEAPONBEHAVIOUR_HPP
#define	WEAPONBEHAVIOUR_HPP

class WeaponBehaviour {
public:
  /// @brief destructor
  virtual ~WeaponBehaviour();
    
  /// @brief function that decides to use weapon
  /// @return -
  virtual void useWeapon(void) = 0;
};

#endif

/* 
 * File:   Warrior.h
 * Author: boo
 *
 * Created on November 2, 2013, 11:29 PM
 */

#ifndef CHARACTER_HPP
#define	CHARACTER_HPP

#include "WeaponBehaviour.h"
#include <string>

class Warrior {
public:
  /// @brief constructor
  Warrior(const std::string &name);
  
  /// @brief destructor
  virtual ~Warrior();
    
  /// @brief function that manages fighting
  /// @return -
  virtual void fight(void) = 0;

  /// @brief function that sets weapon of character
  /// @param [in] weapon - weapon behaviour
  /// @return -
  void setWeapon(WeaponBehaviour *weapon);
            
protected:
  //! name
  std::string m_name;
  
  //! weapon
  WeaponBehaviour *m_weapon;
};

#endif

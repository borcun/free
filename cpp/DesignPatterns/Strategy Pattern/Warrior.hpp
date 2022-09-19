/* 
 * File:   Warrior.hpp
 * Author: boo
 *
 * Created on November 2, 2013, 11:29 PM
 */

#ifndef CHARACTER_HPP
#define	CHARACTER_HPP

#include "WeaponBehaviour.hpp"

namespace Character {
    class Warrior {
        public:
            /// @brief function that manages fighting
            /// @return -
            virtual void fight(void) = 0;

            /// @brief function that sets weapon of character
            /// @param [in] weapon - weapon behaviour
            /// @return -
            void setWeapon(Weapon::WeaponBehaviour *weapon) {
                m_weapon = weapon;
            }
            
        protected:
            Weapon::WeaponBehaviour *m_weapon;
    };
}

#endif


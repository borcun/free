/* 
 * File:   SwordBehaviour.hpp
 * Author: boo
 *
 * Created on November 2, 2013, 11:27 PM
 */

#ifndef SWORDBEHAVIOUR_HPP
#define	SWORDBEHAVIOUR_HPP

#include "WeaponBehaviour.hpp"

namespace Weapon {
    class SwordBehaviour : public WeaponBehaviour {
        public:
            void useWeapon(void) {
                std::cout << "swinging with a sword" << std::endl;
                return;
            }
    };
}

#endif	/* SWORDBEHAVIOUR_HPP */


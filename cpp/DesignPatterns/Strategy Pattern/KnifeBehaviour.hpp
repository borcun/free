/* 
 * File:   KnifeBehaviour.hpp
 * Author: boo
 *
 * Created on November 2, 2013, 11:07 PM
 */

#ifndef KNIFEBEHAVIOUR_HPP
#define	KNIFEBEHAVIOUR_HPP

#include "WeaponBehaviour.hpp"

namespace Weapon {
    class KnifeBehaviour : public WeaponBehaviour {
        public:
            void useWeapon(void) {
                std::cout << "cutting with a knife" << std::endl;
                return;
            }
    };
}

#endif	/* KNIFEBEHAVIOUR_HPP */


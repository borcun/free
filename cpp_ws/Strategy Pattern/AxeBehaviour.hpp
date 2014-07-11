/* 
 * File:   AxeBehaviour.hpp
 * Author: boo
 *
 * Created on November 2, 2013, 11:26 PM
 */

#ifndef AXEBEHAVIOUR_HPP
#define	AXEBEHAVIOUR_HPP

#include "WeaponBehaviour.hpp"

namespace Weapon
{
    class AxeBehaviour : public WeaponBehaviour
    {
    public:
        void useWeapon() {
            std::cout << "chopping with an axe";
            return;
        }
    };
}


#endif	/* AXEBEHAVIOUR_HPP */


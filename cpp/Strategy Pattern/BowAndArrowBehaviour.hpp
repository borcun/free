/* 
 * File:   BowAndArrowBehaviour.hpp
 * Author: boo
 *
 * Created on November 2, 2013, 11:23 PM
 */

#ifndef BOWANDARROWBEHAVIOUR_HPP
#define	BOWANDARROWBEHAVIOUR_HPP

#include "WeaponBehaviour.hpp"

namespace Weapon
{
    class BowAndArrowBehaviour : public WeaponBehaviour
    {
    public:
        void useWeapon() {
            std::cout << "shooting an arrow with a bow";
            return;
        }
    };
}


#endif	/* BOWANDARROWBEHAVIOUR_HPP */


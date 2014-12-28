/* 
 * File:   WeaponBehaviour.hpp
 * Author: boo
 *
 * Created on November 2, 2013, 11:00 PM
 */

#ifndef WEAPONBEHAVIOUR_HPP
#define	WEAPONBEHAVIOUR_HPP

#include <iostream>

/** \brief Weapon Namespace */
namespace Weapon
{
    /** \brief Abstract WeaponBehaviour class*/
    class WeaponBehaviour
    {
    public:
        /// \brief function that decides to use weapon
        /// @return -
        virtual void useWeapon() = 0;
    };
    
}

#endif	/* WEAPONBEHAVIOUR_HPP */


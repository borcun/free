/* 
 * File:   Troll.hpp
 * Author: boo
 *
 * Created on November 2, 2013, 11:36 PM
 */

#ifndef TROLL_HPP
#define	TROLL_HPP

#include "Warrior.hpp"

namespace Character {
    class Troll : public Warrior {
        public:
            Troll(const std::string &name) {
                m_name = name;
            }
            
            void fight(void) {
                std::cout << m_name << " "; 
                m_weapon->useWeapon();

                return;
            }
            
        private:
            std::string m_name;
    };
}

#endif	/* TROLL_HPP */


/**
  * @file King that uses bow and arrow
  * @author boo
  * @date November 2, 2013
  */
#ifndef KING_HPP
#define KING_HPP

#include "Warrior.hpp"

namespace Character
{
    class King : public Warrior
    {
    public:
        King(std::string name) {
            m_name = name;
        }

        void fight() {
            std::cout << m_name << " ";
            m_weapon->useWeapon();
            std::cout << std::endl;
            return;
        }

    private:
        std::string m_name;
    };
}


#endif // KING_HPP

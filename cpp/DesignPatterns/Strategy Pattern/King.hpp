/**
  * @file King that uses bow and arrow
  * @author boo
  * @date November 2, 2013
  */
#ifndef KING_HPP
#define KING_HPP

#include "Warrior.hpp"

namespace Character {
    class King : public Warrior {
        public:
            King(const std::string &name) {
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


#endif // KING_HPP

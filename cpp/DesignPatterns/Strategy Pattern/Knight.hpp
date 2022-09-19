#ifndef KNIGHT_HPP
#define KNIGHT_HPP

#include "Warrior.hpp"

namespace Character {
    class Knight : public Warrior {
        public:
            Knight(const std::string &name) {
                m_name = name;
            }

            void fight() {
                std::cout << m_name << " ";
                m_weapon->useWeapon();

                return;
            }

        private:
            std::string m_name;
    };
}

#endif // KNIGHT_HPP

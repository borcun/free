#ifndef QUEEN_HPP
#define QUEEN_HPP

#include "Warrior.hpp"

namespace Character {
    class Queen : public Warrior {
        public:
            Queen(const std::string &name) {
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

#endif // QUEEN_HPP

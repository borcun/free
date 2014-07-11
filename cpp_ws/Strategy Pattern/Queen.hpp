#ifndef QUEEN_HPP
#define QUEEN_HPP

#include "Warrior.hpp"

namespace Character
{
    class Queen : public Warrior
    {
    public:
        Queen(std::string name) {
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

#endif // QUEEN_HPP

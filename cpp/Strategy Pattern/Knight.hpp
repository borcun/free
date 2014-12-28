#ifndef KNIGHT_HPP
#define KNIGHT_HPP

#include "Warrior.hpp"

namespace Character
{
    class Knight : public Warrior
    {
    public:
        Knight(std::string name) {
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

#endif // KNIGHT_HPP

/**
  * @file main
  * @title strategy pattern
  * @author boo
  * @date November 2, 2013, 10:32 PM
  */

#include "Troll.hpp"
#include "King.hpp"
#include "Queen.hpp"
#include "Knight.hpp"
#include "AxeBehaviour.hpp"
#include "BowAndArrowBehaviour.hpp"
#include "KnifeBehaviour.hpp"
#include "SwordBehaviour.hpp"

#define WARRIOR_COUNT (4)
#define WEAPON_COUNT  (WARRIOR_COUNT)

using namespace Weapon;
using namespace Character;

int main(int argc, char **argv) 
{
  Warrior *warriors[WARRIOR_COUNT] = {
    new Troll("Troll"),
    new King("King"),
    new Queen("Queen"),
    new Knight("Knight")
  };

  WeaponBehaviour *weapons[WEAPON_COUNT] = {
    new AxeBehaviour(),
    new BowAndArrowBehaviour(),
    new KnifeBehaviour(),
    new SwordBehaviour()
  };

  for (int i = 0; i < WARRIOR_COUNT; ++i) {
    warriors[i]->setWeapon(weapons[i]);
  }

  for (int i = 0; i < WARRIOR_COUNT; ++i) {
    warriors[i]->fight();
  }

  for (int i = 0; i < WEAPON_COUNT; ++i) {
    delete weapons[i];
  }

  for (int i = 0; i < WARRIOR_COUNT; ++i) {
    delete warriors[i];
  }
  
  return 0;
}


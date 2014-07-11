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

using namespace Weapon;
using namespace Character;

int main(int argc, char** argv) 
{
    Warrior *troll  = new Troll("Troll");
    Warrior *king   = new King("King");
    Warrior *queen  = new Queen("Queen");
    Warrior *knight = new Knight("Knight");

    troll->setWeapon(new AxeBehaviour());
    king->setWeapon(new BowAndArrowBehaviour());
    queen->setWeapon(new KnifeBehaviour());
    knight->setWeapon(new SwordBehaviour());

    troll->fight();
    king->fight();
    queen->fight();
    knight->fight();

    return 0;
}


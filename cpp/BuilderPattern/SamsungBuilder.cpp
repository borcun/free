#include "SamsungBuilder.hpp"

SamsungBuilder::SamsungBuilder() {
   phone.setName("Samsung Note 20");
}

void SamsungBuilder::buildScreen() {
   phone.setScreenSize("2560x1080");
}

void SamsungBuilder::buildColor() {
   phone.setColor("White");
}

void SamsungBuilder::buildCamera() {
   phone.setCamera("48MP");
}

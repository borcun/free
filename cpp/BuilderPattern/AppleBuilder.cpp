#include "AppleBuilder.hpp"

AppleBuilder::AppleBuilder() {
   phone.setName("Apple iPhone 14 Pro Max");
}

void AppleBuilder::buildScreen() {
   phone.setScreenSize("1920x1080");
}

void AppleBuilder::buildColor() {
   phone.setColor("Black");
}

void AppleBuilder::buildCamera() {
   phone.setCamera("32MP");
}

#include "Phone.h"

std::ostream &operator<<(std::ostream &os, const Phone &phone) {
   os << " Name   : " << phone.name << std::endl
   << " Screen : " << phone.screenSize << std::endl
   << " Color  : " << phone.color << std::endl
   << " Camera : " << phone.camera;
   
   return os;
}

Phone::Phone(void) : Phone("unset") {
}

Phone::Phone(const std::string &name) {
   this->name = name;
   screenSize = "unset";
   color = "unset";
   camera = "unset";
}

Phone::~Phone() {
   
}

void Phone::setName(const std::string &name) {
   this->name = name;
   return;
}

void Phone::setScreenSize(const std::string &screenSize) {
   this->screenSize = screenSize;
   return;
}

void Phone::setColor(const std::string &color) {
   this->color = color;
   return;
}

void Phone::setCamera(const std::string &camera) {
   this->camera = camera;
   return;
}

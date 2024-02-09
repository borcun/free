#include "subscriber.h"

Subscriber::Subscriber(const std::string &name) : m_name(name) {

}

Subscriber::~Subscriber() {

}

std::string Subscriber::getName(void) const {
  return m_name;
}

bool Subscriber::operator==(const Subscriber &subscriber) {
  return subscriber.m_name == m_name;
}

bool Subscriber::operator!=(const Subscriber &subscriber) {
  return !(subscriber.m_name == m_name);
}

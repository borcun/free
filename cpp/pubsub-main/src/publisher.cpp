#include "publisher.h"

Publisher::Publisher(void) : m_name("publisher") {

}

Publisher::Publisher(const std::string &name) : m_name(name) {

}

Publisher::~Publisher() {
  m_subscribers.clear();
}

void Publisher::setName(const std::string &name) {
  m_name = name;
  return;
}

std::string Publisher::getName(void) const {
  return m_name;
}

bool Publisher::subscribe(Subscriber *subscriber) {
  if (nullptr == subscriber) {
    return false;
  }
  
  std::list<Subscriber *>::iterator it;
  bool isAdded = false;
    
  for (it = m_subscribers.begin(); it != m_subscribers.end() && !isAdded; ++it) {
    if (subscriber->getName() == (*it)->getName()) {
      isAdded = true;
    }
  }

  if (!isAdded) {
    m_subscribers.push_back(subscriber);
    return true;
  }

  return false;
}

void Publisher::notify(Message &message) {
  std::list<Subscriber *>::iterator it;
    
  for (it = m_subscribers.begin(); it != m_subscribers.end(); ++it) {
    (*it)->onReceived(message);
  }

  return;
}

#include "topic.h"

unsigned int Topic::count = 0;

std::ostream& operator<<(std::ostream &os, const Topic &topic) {
  os <<
    "   [Topic Id] " << topic.m_id << std::endl <<
    "[Subscribers] ";

  for (auto const &sub : topic.m_subscribers) {
    os << sub->getName() << " ";
  }

  os << "(" << topic.m_subscribers.size() << ")" << std::endl;
  
  return os;
}

Topic::Topic(void) {
  m_id = ++count;
}

Topic::Topic(const Topic &topic) {
  /*
   * the following copy is shallow copy, but instances are created by user
   * thus their life scopes are dependent to user, not to us 
   */
  for (auto const &sub : topic.m_subscribers) {
    m_subscribers.push_back(sub);
  }

  m_id = topic.m_id;
}

Topic::~Topic() {
  m_subscribers.clear();
}

unsigned int Topic::getId(void) {
  return m_id;
}

bool Topic::subscribe(Subscriber *subscriber) {
  std::list<Subscriber *>::iterator it;
  bool isSubscribed = false;
    
  for (it = m_subscribers.begin(); it != m_subscribers.end() && !isSubscribed; ++it) {
    if (subscriber == *it) {
      isSubscribed = true;
    }
  }

  if (!isSubscribed) {
    m_subscribers.push_back(subscriber);
  }

  return !isSubscribed;
}

bool Topic::unsubscribe(Subscriber *subscriber) {
  std::list<Subscriber *>::iterator it;
  bool isSubscribed = false;
    
  for (it = m_subscribers.begin(); it != m_subscribers.end() && !isSubscribed; ++it) {
    if (subscriber == *it) {
      isSubscribed = true;
    }
  }

  if (isSubscribed) {
    m_subscribers.remove(*it);
  }
  
  return isSubscribed;
}

void Topic::publish(const Message &message) {
  std::unique_lock lock(m_mutex);

  for (auto const &sub : m_subscribers) {
    sub->onReceived(message);
  }
  
  return;
}

bool Topic::operator==(const Topic &topic) {
  return m_id == topic.m_id;
}

bool Topic::operator!=(const Topic &topic) {
  return !(*this == topic);
}

#ifndef SUBSCRIBER_H
#define SUBSCRIBER_H

#include "message.h"

class Subscriber {
 public:
  Subscriber(void);
  Subscriber(const std::string &name);
  virtual ~Subscriber();
  void setName(const std::string &name);
  std::string getName(void) const;
  virtual void onReceived(Message &message) = 0;

 private:
  std::string m_name;
};

#endif

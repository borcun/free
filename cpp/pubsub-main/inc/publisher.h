#ifndef PUBLISHER_H
#define PUBLISHER_H

#include "subscriber.h"
#include <list>

class Publisher {
 public:
  Publisher(void);
  Publisher(const std::string &name);
  virtual ~Publisher();
  void setName(const std::string &name);
  std::string getName(void) const;
  virtual bool subscribe(Subscriber *subscriber) final;
  virtual void notify(Message &message) final;
  
 private:
  std::string m_name;
  std::list<Subscriber *> m_subscribers;
};

#endif

#ifndef RSS_READER_H
#define RSS_READER_H

#include "subscriber.h"

class RSSReader : public Subscriber {
 public:
  RSSReader(const std::string &name);
  ~RSSReader();
  virtual void onReceived(const Message &message);
};

#endif

#ifndef MESSAGE_SERVICE_H
#define MESSAGE_SERVICE_H

#include "message.h"
#include "subscriber.h"
#include "publisher.h"

class MessageService {
 public:
  MessageService(const std::string &name);
  virtual ~MessageService();
  void setMessage(Message *message);
  void setPublisher(Publisher *publisher);
  bool subscribe(Subscriber *subscriber);
  bool execute(void);

 private:
  std::string m_name;
  Message *m_message;
  Publisher *m_publisher;
};

#endif

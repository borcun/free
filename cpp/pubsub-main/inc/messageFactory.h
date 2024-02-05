/**
 * @file messageFactory.h
 * @brief the message factory class is implemented as a factory method creating unique message instance
 */

#ifndef MESSAGE_FACTORY_H
#define MESSAGE_FACTORY_H

#include <list>
#include "message.h"

class MessageFactory {
 public:
  static MessageFactory *getInstance(void);
  virtual ~MessageFactory();
  Message createMessage(const std::string &topic);
  bool contain(const Message &message);

 private:
  static MessageFactory *m_instance;
  std::list<Message> m_messageList;
  
  MessageFactory(void) = default;
  MessageFactory(const MessageFactory &messageFactory) = delete;
  MessageFactory &operator=(const MessageFactory &messageFactory) = delete;
};

#endif

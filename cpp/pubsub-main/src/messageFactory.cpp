#include "messageFactory.h"

MessageFactory *MessageFactory::m_instance = nullptr;

MessageFactory::~MessageFactory() {
  if (nullptr != m_instance) {
    delete m_instance;
  }
}

MessageFactory *MessageFactory::getInstance(void) {
  if (nullptr == m_instance) {
    m_instance = new MessageFactory();
  }

  return m_instance;
}

Message MessageFactory::createMessage(const std::string &topic) {
  Message message {topic};

  if (!contain(message)) {
    m_messageList.push_back(message);
  }

  return message;
}

bool MessageFactory::contain(const Message &message) {
  std::list<Message>::iterator it;
  bool isContained = false;
  
  for (it = m_messageList.begin(); it != m_messageList.end() && !isContained; ++it) {
    if (message.getTopic() == it->getTopic()) {
      isContained = true;
    }
  }

  return isContained;
}

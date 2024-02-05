#include "messageService.h"

MessageService::MessageService(const std::string &name) : m_name(name) {
  m_message = nullptr;
  m_publisher = nullptr;
}

MessageService::~MessageService() {

}

void MessageService::setMessage(Message *message) {
  m_message = message;
  return;
}

void MessageService::setPublisher(Publisher *publisher) {
  m_publisher = publisher;
  return;
}

bool MessageService::subscribe(Subscriber *subscriber) {
  bool isAdded = false;
  
  if (nullptr != m_publisher) {
    isAdded = m_publisher->subscribe(subscriber);
  }

  return isAdded;
}

bool MessageService::execute(void) {
  if (nullptr == m_publisher || nullptr == m_message) {
    return false;
  }

  m_publisher->notify(*m_message);
  return true;
}

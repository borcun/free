#include "message.h"

std::ostream& operator<<(std::ostream &os, const Message &message) {
  os <<
    "   [Time] " << message.m_tv.tv_sec << "." << message.m_tv.tv_usec << std::endl <<
    "[Payload] " << message.m_payload << std::endl;

  return os;
}

Message::Message(void) : m_payload("") {
  m_tv.tv_sec = 0;
  m_tv.tv_usec = 0;
}

Message::Message(const std::string &payload) : m_payload(payload) {
  m_tv.tv_sec = 0;
  m_tv.tv_usec = 0;
}

Message::Message(const Message &message) {
  m_tv = message.m_tv;
  m_payload = message.m_payload;
}

Message::Message(const Message &&message) {
  m_tv = std::move(message.m_tv);
  m_payload = std::move(message.m_payload);
}

Message::~Message() {
  m_payload.clear();
}

void Message::setTime(const struct timeval &tv) {
  m_tv = tv;
  return;
}

void Message::setPayload(const std::string &payload) {
  m_payload = payload;  
  return;
}

struct timeval Message::getTime(void) const {
  return m_tv;
}

std::string Message::getPayload(void) const {
  return m_payload;
}

Message &Message::operator=(const Message &message) {
  m_tv = message.m_tv;
  m_payload = message.m_payload;

  return *this;
}

Message &Message::operator=(const Message &&message) {
  m_tv = std::move(message.m_tv);
  m_payload = std::move(message.m_payload);

  return *this;
}

bool Message::operator==(const Message &message) {
  return m_tv.tv_sec == message.m_tv.tv_sec &&
    m_tv.tv_usec == message.m_tv.tv_usec &&
    m_payload == message.m_payload;
}

bool Message::operator!=(const Message &message) {
  return !(*this == message);
}

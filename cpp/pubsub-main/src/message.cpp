#include "message.h"

std::ostream& operator<<(std::ostream &os, const Message &message) {
  os <<
    "  [Topic] " << message.m_topic << std::endl <<
    "   [Time] " << message.m_time.tv_sec << "." << message.m_time.tv_usec << std::endl <<
    "[Payload] " << message.m_payload << std::endl;

  return os;
}

Message::Message(const std::string &topic) : m_topic(topic) {
  m_time.tv_sec = 0;
  m_time.tv_usec = 0;
  m_payload.clear();
}

Message::Message(const Message &message) {
  m_topic = message.m_topic;
  m_time = message.m_time;
  m_payload = message.m_payload;
}

Message::Message(const Message &&message) {
  m_topic = std::move(message.m_topic);
  m_time = std::move(message.m_time);
  m_payload = std::move(message.m_payload);
}

Message::~Message() {
  m_topic.clear();
  m_payload.clear();
}

void Message::setPayload(const std::string &payload) {
  m_payload = payload;
  // once payload is set, its time label is also set
  gettimeofday(&m_time, NULL);
  
  return;
}

std::string Message::getTopic(void) const {
  return m_topic;
}

struct timeval Message::getTime(void) const {
  return m_time;
}

std::string Message::getPayload(void) const {
  return m_payload;
}

Message &Message::operator=(const Message &message) {
  m_topic = message.m_topic;
  m_time = message.m_time;
  m_payload = message.m_payload;

  return *this;
}

Message &Message::operator=(const Message &&message) {
  m_topic = std::move(message.m_topic);
  m_time = std::move(message.m_time);
  m_payload = std::move(message.m_payload);

  return *this;
}

bool Message::operator==(const Message &message) {
  return m_topic == message.m_topic && m_payload == message.m_payload;
}

bool Message::operator!=(const Message &message) {
  return !(*this == message);
}

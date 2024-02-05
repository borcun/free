#include "rssReader.h"

RSSReader::RSSReader(const std::string &name) : Subscriber(name) {

}

RSSReader::~RSSReader() {

}

void RSSReader::onReceived(const Message &message) {
  std::cout << "Received message by " << getName() << std::endl;
  std::cout << message << std::endl;
}

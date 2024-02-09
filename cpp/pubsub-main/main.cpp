#include "messageFactory.h"
#include "publisher.h"
#include "subscriber.h"
#include "messageService.h"
#include <unistd.h>

#define MESSAGE_TOPIC_1  ("rss-topic-1")
#define MESSAGE_TOPIC_2  ("rss-topic-2")
#define MESSAGE_PAYLOAD_1 ("This is my first RSS payload")
#define MESSAGE_PAYLOAD_2 ("This is my second RSS payload")
#define MESSAGE_SERVICE   ("message-service")
#define MESSAGE_PUBLISHER ("rss-publisher")
#define MESSAGE_SUBSCRIBER_1 ("rss-subscriber-1")
#define MESSAGE_SUBSCRIBER_2 ("rss-subscriber-2")

class RSSPublisher : public Publisher {
public:
  RSSPublisher(void) : Publisher() {}
  RSSPublisher(const std::string &name) : Publisher(name) {}
  ~RSSPublisher() {}
};

class RSSReader : public Subscriber {
public:
  RSSReader(void) : Subscriber() {}
  RSSReader(const std::string &name) : Subscriber(name) {}
  ~RSSReader() {}

  virtual void onReceived(Message &message) {
    std::cout << "Received message by " << getName() << std::endl;
    std::cout << message << std::endl;
  }
};

int main() {
  MessageFactory *messageFactory = MessageFactory::getInstance();
  Message message = messageFactory->createMessage(MESSAGE_TOPIC_1);
  MessageService messageService {MESSAGE_SERVICE};
  RSSPublisher publisher {MESSAGE_PUBLISHER};
  RSSReader subscriber1 {MESSAGE_SUBSCRIBER_1};
  RSSReader subscriber2 {MESSAGE_SUBSCRIBER_2};

  message.setPayload(MESSAGE_PAYLOAD_1);

  messageService.setMessage(&message);
  messageService.setPublisher(&publisher);
  messageService.subscribe(&subscriber1);

  messageService.execute();

  sleep(2);
  messageService.subscribe(&subscriber2);
  message.setPayload(MESSAGE_PAYLOAD_2);

  messageService.execute();
  
  return 0;
}

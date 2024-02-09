### About Library

It is a small publisher-subscriber library, which is developed on Fedora OS for AMD x64 architecture.
The library includes four important components:

 1. Message
 2. Topic
 3. TopicFactory
 4. Publisher & Subscriber

Message is the basic data model, which exists time label and payload fields. The time label is struct 
timeval instance of the any UNIX system. It is used to reveal the time when the payload of Message 
instance is set. Therefore, it should be set once the payload is set. The payload is data part of 
Message instance.

Topic is a context which includes subscribers and are connected to many publishers. Each topic
instance has unique ID value that is used to store topics created by TopicFactory. Since publishers
more than one may have merely one topic, topic functionality must be thread-safe. Its publishing
function is protected by using locking mechanism.

TopicFactory is a class that is designed as both factory and singleton design patterns. It generates
topics, and stores them into a topic list to prevent regeneration. Therefore, it can provides 
uniqueness of each message.

Publisher and Subscriber classes are communication components of the library. Knowing from observer
design pattern, these components provides event-based communication instead of polling mechanism.
Sharing data between them is Message instances.

Publisher and Subscriber classes are interfaces for the communication. Although Subscriber is an
abstract class in terms of C++ terminology, Publisher is concrete class. Developer can use Publisher
class as it's present, but inheriting it as well like does for Subscriber classes is recommended.
Subscriber class includes a pure virtual function called onReceived. Its aim is being used as a
callback by developer.

Example directory includes a simple example to demonstrate how library works. Test directory
includes some test steps like unit test. Due to dependency conflict or missing reference consideration,
any 3rd party test framework is not used.

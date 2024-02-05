# Lock-Free Circular Queue

Lock-Free Circular Queue is a queue including the lock-free mechanism for enqueuing and dequeuing operations. It is used on consumer-producer scenarios, which need no locking rules, therefore no latency occurs. The concurrent access problem is solved using atomic operations.

# Implementation Details

The project is implemented using the help of atomic concept of the C++ programming language. In case of enqueuing and dequeuing occurring at same time, resource, namely the queue may likely cause a race condition. Meanwhile, enqueuing and dequeuing are competitors of each other, they must be synchronized. However, the latency mostly occurs when using a locking mechanism, because dequeuing and enqueuing have to wait for each other. The atomic concept solves concurrent access without any latency, but for the sake of performance,  some data-lost may occur.

The head, tail, and size fields of the lock-free circular buffer are atomic variables. An atomic operation is generally implemented using the special mechanism of the operating system such as XCHG in Intel Architecture or TSL in another operating system. As understood by name of the atomic operation, any arithmetic operation performed on an atomic variable can not interrupt by another process, or interrupt routine, so it's similar to the semaphore in terms of its down and up operations.


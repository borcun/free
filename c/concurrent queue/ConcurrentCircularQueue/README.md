# Concurrent Queue
Concurrent Queue is a queue that circulates head and tail pointers when they reach end of the queue.
Thus, the queue never becomes full. However, some implementations may check queue size whether it is full
or not via checking tail pointer that reaches end of the queue. In this implementation, there is no check
buffer fullness.

Concurrent Queue has four functions: init, deinit, enqueue and dequeue. init and deinit are so easy-understand
functions as reading their names, but enqueue and dequeue are thoroughly specific notions with respect to queue 
like push, pop or front. The enqueue inserts an element at index where indicated by the tail, while the dequeue 
gets element at index where indicated by the head.

The main function has two different test scenarios. The first one is checking all functions with simple test
procedures. The second one is a bit more sophisticated test step. In that test, two threads, consumer and 
producer, are created, and executed concurrently. They compete for enqueuing and dequeuing on the circular 
buffer. To prevent access by two threads at same time, a mutex is used to lock and unlock the circular buffer. 
The competition steps are printed at each operation with + for enqueuing and - for dequeuing. To see the results
clearly, enqueuing and dequeuing counts may be increased giving two parameters to the executable file. 

Note that more modern processor architectures can perform the operations without switching between threads or 
with fewer switchings. To more clear outputs, please use big count values.

**Development Environment:** <br>
Apple clang version 13.0.0 (clang-1300.0.29.30) <br>
Target: arm64-apple-darwin21.2.0

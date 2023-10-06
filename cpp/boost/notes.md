## RAII (Resource Acquisition is Initialization)
   RAII is a method that refers to an acquisition which is also initialization for a resource.
   The best example is scope lock mutex, smart pointer or thread. When you initialize one of them,
   it is also a resource acquisition at same time. Thus, you don't need one more function call too
   after the initialization phase. They help avoid memory leaks.

   https://en.cppreference.com/w/cpp/language/raii
   
## Smart Pointers
   The boost smart pointer library provides 5 smart pointer class templates.
   scoped_ptr, scoped_array, shared_ptr, shared_array, weak_ptr, intrusive_ptr
   
   scoped_ptr can't transfer ownership of an object. Once initialized with an
   address, the dynamically allocated object is release when the destructor is
   execute or when the member function reset() is called.


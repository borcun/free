socket function creates a communication endpoint and returns a socket number whose type is integer.
in fact, the socket number is a file descriptor. When creating a socket, programmer give some parameters
into socket function to indicate socket properties. For example, AF_INET stands for address family,
PF_INET stands for protocol family. ( Nowadays,PF_INET == AF_INET )
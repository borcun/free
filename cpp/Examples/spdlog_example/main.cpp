#include <iostream>
#include <spdlog/spdlog.h>

int main() {
  auto console = spdlog::rotating_logger_mt( "basic_logger", "log.txt", 16, 1 );

  console->info( "welcome to spdlog!" );
  console->info( "what\'s app {}?", "boo" );


  return 0;
}

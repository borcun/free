#include <iostream>
#include <string>

class MyString {
  friend std::ostream &operator<<( std::ostream &os, const MyString &str );
  
public:
  MyString( std::string str ) {
    m_str = str;
  }

  std::string getVal( void ) const {
    return m_str;
  }

  MyString &operator+( const MyString &str ) {  
    return *(new MyString( m_str + " " + str.getVal() ));
  }

private:
  std::string m_str;
};

std::ostream &operator<<( std::ostream &os, const MyString &str ) {
  return os << str.m_str;
}

int main() {
  MyString str1( "hello" );
  MyString str2( "boo!" );

  std::cout << ( str1 + str2 ) << std::endl;
  
  return 0;
}

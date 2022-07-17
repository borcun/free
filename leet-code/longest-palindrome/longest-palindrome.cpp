#include <iostream>
#include <string>

class Solution
{
public:
  Solution(void) {
    m_result = "";
  }
  
  std::string longestPalindrome(std::string str) {
    if (0 == str.length() || 1 == str.length()) {
      return str;
    }
      
    if (2 == str.length()) {
      return str[0] == str[1] ? str : str.substr(0, 1);
    }
      
    for (int i = 0; i < str.length(); ++i) {
      for (int j = str.length() - i; j > 1; --j) {
	std::string tmp = str.substr(i, j);
      
	if (isPalindrome(tmp)) {
	  if (tmp.length() > m_result.length()) {
	    m_result = tmp;
	  }
	}
      }
    }
      
    if (0 == m_result.length()) {
      return str.substr(0, 1);
    }

    return m_result;
  }

private:
  std::string m_result;
  
  bool isPalindrome(std::string str) {
    return palindromeUtil(str, 0, str.length() - 1);
  }

  bool palindromeUtil(std::string str, int begin, int end) {
    if (begin > end) {
      return false;
    }

    if ((begin == end) || (end - begin == 1)) {
      return str[begin] == str[end];
    }

    return str[begin] == str[end] ? palindromeUtil(str, begin + 1, end - 1) : false;
  }
};

int main() {
  Solution solution;
  std::string str = "ibvjkmpyzsifuxcabqqpahjdeuzaybqsrsmbfplxycsafogotliyvhxjtkrbzqxlyfwujzhkdafhebvsdhkkdbhlhmaoxmbkqiwiusngkbdhlvxdyvnjrzvxmukvdfobzlmvnbnilnsyrgoygfdzjlymhprcpxsnxpcafctikxxybcusgjwmfklkffehbvlhvxfiddznwumxosomfbgxoruoqrhezgsgidgcfzbtdftjxeahriirqgxbhicoxavquhbkaomrroghdnfkknyigsluqebaqrtcwgmlnvmxoagisdmsokeznjsnwpxygjjptvyjjkbmkxvlivinmpnpxgmmorkasebngirckqcawgevljplkkgextudqaodwqmfljljhrujoerycoojwwgtklypicgkyaboqjfivbeqdlonxeidgxsyzugkntoevwfuxovazcyayvwbcqswzhytlmtmrtwpikgacnpkbwgfmpavzyjoxughwhvlsxsgttbcyrlkaarngeoaldsdtjncivhcfsaohmdhgbwkuemcembmlwbwquxfaiukoqvzmgoeppieztdacvwngbkcxknbytvztodbfnjhbtwpjlzuajnlzfmmujhcggpdcwdquutdiubgcvnxvgspmfumeqrofewynizvynavjzkbpkuxxvkjujectdyfwygnfsukvzflcuxxzvxzravzznpxttduajhbsyiywpqunnarabcroljwcbdydagachbobkcvudkoddldaucwruobfylfhyvjuynjrosxczgjwudpxaqwnboxgxybnngxxhibesiaxkicinikzzmonftqkcudlzfzutplbycejmkpxcygsafzkgudy";

  std::cout << "Longest Palindrome: " << solution.longestPalindrome(str) << std::endl;
  return 0;
}

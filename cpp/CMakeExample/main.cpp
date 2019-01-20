#include <iostream>
#include <string>
#include <vector>
#include <string.h>

size_t filter(void* destination,
	      const void* source,
              size_t source_size,
              size_t history_size)
{
    // avoid from unexpected arguments
    if( NULL == destination || NULL == source || 0 == source_size ) {
        // give an error
        return 0;
    }
    
    char *des = ( char * ) destination;
    char *src = ( char * ) source;
    int des_cnt = 0;
    
    memset( des, '\0', source_size );
    des[0] = src[0];
    ++des_cnt;
    
    // do not check size of source and destination because that's guaranted their sizes are bigger than #source_size   
    for( int i=0, j=1; i + j < source_size; ++i ) {
        int des_len = ( j < history_size ? j : history_size );
        int start_index = ( j > history_size ? (j-history_size) : 0 );
            
        for( int k=start_index ; k < des_len; ++k ) {
            if( des[k] != src[k] ) {
                des[j-1] = src[k];
                ++j;
            }
        }
        
        ++j;
    }
    
    return des_cnt;
}

//
// Filter string. Helper for mapping binary blobs to strings.
//
std::string filter_string(const std::string& source, size_t history_size)
{
    std::vector<char> buffer(source.length());
    size_t filtered_size = filter(buffer.data(), source.c_str(), source.length(), history_size);
    std::string dest(buffer.data(), filtered_size);
    return dest;
}

//
// Main entry point.
//
int main()
{
    while (!std::cin.eof())
    {
        size_t history_size = 0;
        std::cin >> history_size;
        std::string source;
        std::getline(std::cin, source);
        std::cout << filter_string(source, history_size) << std::endl;
    }
    return 0;
}

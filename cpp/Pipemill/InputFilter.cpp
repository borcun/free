#include "InputFilter.hpp"

using namespace Pipemill;

// constructor
InputFilter::InputFilter(enum mode m) : BaseFilter(m), m_local_index(-1)
{
    
}

// destructor
InputFilter::~InputFilter()
{
    m_local_index = -1;
}

// function that is called by TBB automatically
// It calls process function to process data and
// returns index number that it process after how many
void *InputFilter::operator()(void* index)
{                     
    // call process function
    process(index);
    
    // increase m_local_index and mod it by CIRCULAR_SIZE
    if(++m_local_index == CIRCULAR_SIZE)
        m_local_index %= CIRCULAR_SIZE;
    
    return &m_local_index;
}

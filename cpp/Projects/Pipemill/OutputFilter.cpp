#include "OutputFilter.hpp"

using namespace Pipemill;

// constructor
OutputFilter::OutputFilter(enum mode m) : BaseFilter(m)
{
    
}

// destructor
OutputFilter::~OutputFilter()
{
    
}

// function that is called by TBB automatically
// It calls process function to process data and
// returns index number that it process after how many
void *OutputFilter::operator ()(void* index)
{                 
    // call process function
    process(index);
    
    return index;
}

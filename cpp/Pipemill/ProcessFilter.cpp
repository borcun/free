#include "ProcessFilter.hpp"

using namespace Pipemill;

// constructor
ProcessFilter::ProcessFilter(enum mode m) : BaseFilter(m)
{
    
}

// destructor
ProcessFilter::~ProcessFilter()
{

}

// function that is called by TBB automatically
// It calls process function to process data and
// returns index number that it process after how many
void *ProcessFilter::operator()(void* index)
{      
    // call process function
    process(index);
    
    return index;
}

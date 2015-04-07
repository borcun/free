#include "Pipeline.hpp"

using namespace Pipemill;

// utility function that runs pipeline
void runPipemill(Pipeline *ab_pipeline, const size_t max_number_of_live_tokens)
{    
    ab_pipeline->run(max_number_of_live_tokens);
}

// constructor
// when construct object, user must tell how pipeline is running
// multi_pipeline parameter specify it. If it is false (default), pipeline runs single.
// If the parameter is true, this pipeline runs parallel with other pipeline used by multi pipeline.
Pipeline::Pipeline(const bool multi_pipeline) : m_is_running(false), m_is_multi(multi_pipeline)
{
    tbb::task_scheduler_init init;
}

// destructor
Pipeline::~Pipeline()
{
    m_is_running = false;
    m_is_multi = false;
}

// function that runs pipeline
// It creates a boost thread and give an duty that is runs pipeline with this pointer
// Before running, check if pipeline is running or not.
void Pipeline::runPipeline(const size_t max_number_of_live_tokens)
{
    // if pipeline is not running, run pipeline
    if(!m_is_running) {
        // set m_is_running with true
        m_is_running = true;
        // call thread function to run pipeline
        m_thread[0] = boost::thread(runPipemill, this, max_number_of_live_tokens);
        
        // check if pipeline is parallel or not
        if(!m_is_multi)
            m_thread[0].join();
    }
    // if calling function when pipeline is running, give an error message
    else {
        std::cerr << "Pipeline is running !" << std::endl;
    }
    
    return;    
}

// function that adds filters to pipeline
void Pipeline::addFilter(BaseFilter &basefilter)
{
    // add filter parameter to pipeline
    this->add_filter(basefilter);
    
    return;
}

// function that gets pipeline thread id
boost::thread *Pipeline::getThreadID()
{ 
    return m_thread;
}

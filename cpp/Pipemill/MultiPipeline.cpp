#include "MultiPipeline.hpp"

using namespace Pipemill;

// initialize static singleton object with NULL
MultiPipeline *MultiPipeline::m_multi_pipeline = NULL;

// function that runs all pipelines
// It takes pipeline pointers vector and is controlled by boost thread 
void runPipemill(std::vector<Pipeline *> pipelines)
{
    // get pipelines count
    const int size = (int)pipelines.size();
    
    // prepare to start all pipelines
    for(int i=0 ; i < size ; ++i) {
        pipelines[i]->runPipeline(1);        
    }

    // run all pipelines
    for(int i=0 ; i < size ; ++i) {    
        pipelines[i]->getThreadID()->join();
    }
    
    return;
}

// singleton function that gets ABMulti Pipeline object
MultiPipeline *MultiPipeline::getInstance()
{
    tbb::task_scheduler_init init;
        
    // if class object is not instanced, allocate memory
    if(!m_multi_pipeline)
        m_multi_pipeline = new MultiPipeline();
    
    // return object
    return m_multi_pipeline;
}

// default constructor
MultiPipeline::MultiPipeline()
{
}

// destructor
MultiPipeline::~MultiPipeline()
{
    delete m_multi_pipeline;
}

// function that runs all pipeline in pipeline vector
void MultiPipeline::run()
{
    // give an multi thread processing duty to thread 
    m_thread[0] = boost::thread(runPipemill, m_pipeline_vector);
    // run thread
    m_thread[0].join();
    
    return;
}

// function that adds pipeline to pipeline vector
void MultiPipeline::addPipeline(Pipeline *pipeline)
{
    m_pipeline_vector.push_back(pipeline);
    
    return;
}

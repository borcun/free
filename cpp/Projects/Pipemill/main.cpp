/* 
 * File:   main.cpp
 * Author: boo
 * Created on January 8, 2013, 11:54 AM
 */

#include "PipemillTest.hpp"

// main function
int main(int argc, char** argv) 
{       
    // create one pipeline object with false (or defaultly) parameter to run them serial
    Pipeline pipeline1(false);
    // create three filters
    CaptureFilter *capture_filter = new CaptureFilter();
    SobelFilter *sobel_filter = new SobelFilter();
    CannyFilter *canny_filter = new CannyFilter();
    
    // set connections of filters
    // connect out connector of capture filter to in connector of sobel filter
    capture_filter->m_oc->connectTo(sobel_filter->m_in);
    // connect out connector of sobel filter to in connector of canny filter
    sobel_filter->m_oc->connectTo(canny_filter->m_in);

    // add filters to pipeline
    pipeline1.addFilter(*capture_filter);
    pipeline1.addFilter(*sobel_filter);
    pipeline1.addFilter(*canny_filter);

    // run pipeline
    pipeline1.runPipeline(1);
    
    return 0;
}

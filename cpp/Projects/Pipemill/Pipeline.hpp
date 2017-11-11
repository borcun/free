/**
 * @file:    Pipeline.hpp
 * @author:  boo
 * @title:   Pipeline Class derived from tbb::pipeline Class
 * @version: 1.0
 * @date:    January 9, 2013, 5:06 PM
 */

#ifndef PIPELINE_HPP
#define	PIPELINE_HPP

#include <iostream>
#include <boost/thread.hpp>
#include "BaseFilter.hpp"

namespace Pipemill
{
    class Pipeline : public tbb::pipeline
    {        
        public:
            /// \brief constructor
            /// @param multi_pipeline - run type of pipeline
            explicit Pipeline(const bool multi_pipeline = false);
            /// \brief destructor
            virtual ~Pipeline();
            /// \brief function that runs pipeline
            /// @param max_number_of_live_tokens - parameter specifies if pipeline is multi or single
            /// @return -
            void runPipeline(const size_t max_number_of_live_tokens);
            /// \brief function that adds filters to pipeline
            /// @param base_filter - base filter object that is added into pipeline
            /// @return -
            void addFilter(BaseFilter &base_filter);
            /// \brief function that gets pipeline thread id
            /// @return boost thread id
            boost::thread *getThreadID();
            
        private:
            /// \brief thread of pipeline
            boost::thread m_thread[1];
            /// \brief pipeline running flag
            bool m_is_running;   
            /// \brief pipeline processing flag
            bool m_is_multi;

            /// \brief default constructor
            Pipeline();            
    };
}

#endif


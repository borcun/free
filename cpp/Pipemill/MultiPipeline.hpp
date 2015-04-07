/**
 * @file:    MultiPipeline.hpp
 * @author:  boo
 * @title:   Multi Pipeline Class
 * @version: 1.0
 * @date:    January 14, 2013, 3:07 PM
 */

#ifndef MULTIPIPELINE_HPP
#define	MULTIPIPELINE_HPP

#include <vector>
#include "Pipeline.hpp"

namespace Pipemill
{
    class MultiPipeline
    {
        public:
            /// \brief singleton function that gets an instance
            /// @return MultiPipeline pointer
            static MultiPipeline *getInstance();
            /// \brief destructor
            virtual ~MultiPipeline();
            /// \brief function that runs all pipeline in pipeline vector
            /// @return -
            void run();
            /// \brief function that adds pipeline to pipeline vector
            /// @return -
            void addPipeline(Pipeline *);
            
        private:
            /// \brief singleton object
            static MultiPipeline *m_multi_pipeline;
            /// \brief boost thread controls multi pipeline processing
            boost::thread m_thread[1];
            /// \brief pipelines vector
            std::vector<Pipeline *> m_pipeline_vector;
            
            /// \brief default constructor
            MultiPipeline();            
    };
}

#endif


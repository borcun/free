/**
 * @file:    OutputFilter.hpp
 * @author:  boo
 * @title:   Output Filter Class derived from Base Filter Class
 * @version: 1.0
 * @date:    January 11, 2013, 9:32 AM
 */

#ifndef OUTPUTFILTER_HPP
#define	OUTPUTFILTER_HPP

#include "BaseFilter.hpp"

namespace Pipemill
{
    class OutputFilter : public BaseFilter
    {
        public:
            /// \brief constructor
            /// @param mode - run type of filter
            explicit OutputFilter(MODE = serial_in_order);
            /// \brief destructor
            virtual ~OutputFilter();
            /// \brief process function
            /// @param index - index from previous filter
            /// @return -
            virtual void process(void *index) = 0;
            
        protected:
            /// \brief operator function
            /// @param item - item from previous filter
            /// @return item for next filter
            void *operator()(void *item);

        private:
            /// \brief default constructor
            OutputFilter();   
    };
}

#endif


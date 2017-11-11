/**
 * @file:    ProcessFilter.hpp
 * @author:  boo
 * @title:   Process Filter Class derived from Base Filter Class
 * @version: 1.0
 * @date:    January 9, 2013, 11:49 AM
 */

#ifndef PROCESSFILTER_HPP
#define	PROCESSFILTER_HPP

#include "BaseFilter.hpp"

namespace Pipemill
{
    class ProcessFilter : public BaseFilter
    {
        public:
            /// \brief constructor
            /// @param mode - run type of filter
            explicit ProcessFilter(MODE = serial_in_order);
            /// \brief destructor
            virtual ~ProcessFilter();
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
            ProcessFilter();
    };
}

#endif

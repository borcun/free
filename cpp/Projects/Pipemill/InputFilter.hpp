/**
 * @file:    InputFilter.hpp
 * @author:  boo
 * @title:   Input Filter Class derived from Base Filter Class
 * @version: 1.0
 * @date:    January 9, 2013, 5:35 PM
 */

#ifndef INPUTFILTER_HPP
#define	INPUTFILTER_HPP

#include "BaseFilter.hpp"

namespace Pipemill
{
    class InputFilter : public BaseFilter
    {
        public:
            /// \brief constructor
            /// @param mode - run type of filter
            explicit InputFilter(MODE = serial_in_order);
            /// \brief destructor
            virtual ~InputFilter();
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
            /// \brief  local index of circular buffer
            int m_local_index;

            /// \brief default constructor
            InputFilter();            
    };
}

#endif


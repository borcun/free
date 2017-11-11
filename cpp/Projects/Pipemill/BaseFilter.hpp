/**
 * @file:    ABBaseFilter.hpp
 * @author:  boo
 * @title:   Base Filter Class
 * @version: 1.0
 * @date:    January 11, 2013, 9:33 AM
 */

#ifndef BASEFILTER_HPP
#define	BASEFILTER_HPP

#include <tbb/pipeline.h>
#include <tbb/task_scheduler_init.h>

#define CIRCULAR_SIZE 16

namespace Pipemill
{
    class BaseFilter : public tbb::filter
    {
        public:
            /// mode specify filter processing order to parallel, serial_in_order or serial_out_order
            typedef enum mode {
                //! processes multiple items in parallel and in no particular order
                parallel = tbb::filter::current_version | tbb::filter::filter_is_out_of_order, 
                //! processes items one at a time; all such filters process items in the same order
                serial_in_order = tbb::filter::current_version | tbb::filter::filter_is_serial,
                //! processes items one at a time and in no particular order
                serial_out_of_order = tbb::filter::current_version | tbb::filter::filter_is_serial | tbb::filter::filter_is_out_of_order
            } MODE;

            /// \brief constructor
            /// @param mode - run type of filter
            explicit BaseFilter(MODE = serial_in_order) : tbb::filter(serial_in_order) {}
            /// \brief destructor
            virtual ~BaseFilter() {}
            /// \brief process function
            /// @param index - index from previous filter
            /// @return -
            virtual void process(void *index) = 0;
            /// \brief operator function
            /// @param item - item from previous filter
            /// @return item for next filter
            virtual void *operator()(void *item) = 0;

        private:
            /// \brief default constructor
            BaseFilter();
    };
}

#endif


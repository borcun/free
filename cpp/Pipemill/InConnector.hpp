/**
 * @file:    InConnector.hpp
 * @author:  boo
 * @title:   InConnector Class
 * @version: 1.0
 * @date:    January 8, 2013, 12:04 PM
 */

#ifndef INCONNECTOR_HPP
#define INCONNECTOR_HPP

#include <iostream>
#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/circular_buffer.hpp>
#include <boost/unordered_map.hpp>
#include <vector>

#define CIRCULAR_SIZE 16
#define KEY_SIZE 8

namespace Pipemill
{   
    template<class T>
    class InConnector
    {
        public:
            /// \brief default constructor
            InConnector();
            /// \brief destructor
            virtual ~InConnector();
            /// \brief function that sets connection
            /// @param msm_name - managed shared memory name
            /// @param msm_size - managed shared memory size
            /// @return -
            void setConnection(const char *msm_name, const int msm_size);
            /// \brief function that returns all data in its managed shared memory
            /// @param item - data from out connector
            /// @return vector that contains of data(s) picked up from managed shared memory
            std::vector<T> readData(void *item);
            
        private:
            /// \brief managed_shared_memory vector for each out connector
            std::vector< boost::interprocess::managed_shared_memory *> m_msm_vector;
            /// \brief row data is taken from all circular buffer with common index
            std::vector<T> m_row_data_vector;
            /// \brief key of managed_shared_memory
            char m_key[KEY_SIZE];
            /// \brief out connectors count
            int  m_out_connectors_count;
            /// \brief local index
            char m_index[3];            
    };
}

#endif


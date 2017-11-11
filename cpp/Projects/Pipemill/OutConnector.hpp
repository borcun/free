/**
 * @file:    OutConnector.hpp
 * @author:  boo
 * @title:   OutConnector Class
 * @version: 1.0
 * @date:    January 8, 2013, 11:56 AM
 */

#ifndef OUTCONNECTOR_HPP
#define	OUTCONNECTOR_HPP

#include "InConnector.hpp"
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <string>
#include <sstream>

#define MAX_CHAR 64

namespace Pipemill
{    
    /// \brief forward declaration
    template <class T> class InConnector;
    
    template <class T>
    class OutConnector
    {
        public:
            /// \brief constructor that takes managed shared memory size as parameter
            /// @param msm_size - managed shared memory size
            explicit OutConnector(const int msm_size);
            /// \brief destructor
            virtual ~OutConnector();
            /// \brief function that connects in connector to out connector
            /// @param in_connector - in connector object that is same type with out connector
            /// @return -
            void connectTo(InConnector<T> *in_connector);
            /// \brief function that writes data to managed shared memory
            /// @param data - data that will be written in managed shared memory
            /// @return -
            void writeData(const T *data);
            /// \brief function that gets in connectors count connected to out connector
            /// @return int - count of connected in connectors
            inline int getConnectedConnectorsCount() const;
            
        private:
            /// \brief managed shared memory object
            boost::interprocess::managed_shared_memory m_msm;
            /// \brief managed shared memory name
            char *m_msm_name;
            /// \brief managed shared memory key
            char m_key[KEY_SIZE];
            /// \brief managed shared memory size
            int m_msm_size;
            /// \brief index
            int m_index;
            /// \brief counts of connected in connector
            int m_in_connectors_count;
            
            /// \brief utility function that generates special name for managed shared memory
            /// @return -
            void generateName();                               
    };
}

#endif


#include "InConnector.hpp"

using namespace Pipemill;

// default constructor
template <class T>
InConnector<T>::InConnector() : m_out_connectors_count(0)
{
    
}

// destructor
template <class T>
InConnector<T>::~InConnector()
{
    // release managed shared memory
    for(int i=0 ; i < (int)m_msm_vector.size() ; ++i)
        delete m_msm_vector.at(i);

    // clear managed shared memory vector
    m_msm_vector.clear();
    m_out_connectors_count = 0;
}

// function that open shared memory from out connector
// first parameter is name of managed_shared_memory
// second parameter is size of managed_shared_memory
template <class T>
void InConnector<T>::setConnection(const char *msm_name, const int msm_size)
{           
    try {
        // open a managed_shared_memory and push it to back of managed_shared_memory vector
        m_msm_vector.push_back(new boost::interprocess::managed_shared_memory(boost::interprocess::open_or_create, msm_name, msm_size));
        // insert its name and index to map
        //m_msm_map.insert(unordered_map::value_type(msm_name, m_out_connectors_count));
        // add a circular_buffer for out connector
        //m_circular_buffer_vector.push_back(new boost::circular_buffer<T>(CIRCULAR_SIZE));
        // increase connect_out_connectors_count
        ++m_out_connectors_count;
    }
    catch(boost::interprocess::bad_alloc &ex) {
        std::cerr << "ERROR : " << ex.what() << std::endl;
    }
    catch(boost::interprocess::interprocess_exception &ex) {
        std::cerr << "ERROR : " << ex.what() << std::endl;        
    } 
    
    return;
}

// function that returns all data in its buffer
template <class T>
std::vector<T> InConnector<T>::readData(void *item)
{    
    // clear m_index
    memset(m_index, '\0', 3);
    sprintf(m_index, "%d%c", *(static_cast<int *>(item)), '\0');    
    // clear vector
    m_row_data_vector.clear();
    
    // read all data in all circular buffer in this index
    // fill them into a row vector
    for(int i=0 ; i < m_out_connectors_count ; ++i) {
        m_row_data_vector.push_back(*m_msm_vector.at(i)->find<T>(m_index).first);
    }
    
    // return row data vector
    return m_row_data_vector;    
}

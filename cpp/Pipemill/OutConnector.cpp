#include "OutConnector.hpp"
#include "BaseFilter.hpp"

using namespace Pipemill;

// constructor
template <class T>
OutConnector<T>::OutConnector(const int msm_size) : m_index(0), m_in_connectors_count(0)
{   
    // set size of managed shared memory
    m_msm_size = msm_size * sizeof(T);
    // allocate memory for m_msm_name and initialize it
    m_msm_name = new char[MAX_CHAR];
        
    try {
        // get an unique name for managed shared memory
        generateName();
        // remove old shared memory and create managed shared memory
        boost::interprocess::shared_memory_object::remove(m_msm_name); 
        m_msm = boost::interprocess::managed_shared_memory(boost::interprocess::open_or_create, m_msm_name, m_msm_size);
    }
    catch(boost::interprocess::bad_alloc &ex) {
        std::cerr << "ERROR : " << ex.what() << std::endl;
        exit(EXIT_FAILURE);
    }    
    catch(boost::interprocess::interprocess_exception &ex) {
        std::cerr << "ERROR : " << ex.what() << std::endl;        
        exit(EXIT_FAILURE);
    }
}

// destructor
template <class T>
OutConnector<T>::~OutConnector()
{
    // remove managed_shared_memory
    boost::interprocess::shared_memory_object::remove(m_msm_name);
    // delete its name
    delete [] m_msm_name;
    // zero size and index
    m_msm_size = 0;
    m_index = 0;
}

// function that connects shared memory from out connector to in connector
template <class T>
void OutConnector<T>::connectTo(InConnector<T> *in_connector)
{
    try {
        // call setConnection function of in_connector
        // share managed shared memory between out and in connectors
        in_connector->setConnection(m_msm_name, m_msm_size);
    }
    catch(boost::interprocess::bad_alloc &ex) {
        std::cerr << "ERROR : " << ex.what() << std::endl;
        exit(EXIT_FAILURE);

    }
    catch(boost::interprocess::interprocess_exception &ex) {
        std::cerr << "ERROR : " << ex.what() << std::endl;
        exit(EXIT_FAILURE);        
    }
    
    // increase connected_in_connectors_count
    ++m_in_connectors_count;
        
    return;
}

// function that writes data to shared memory
// T *data parameter is data that will be written in shared memory
// size parameter is number that is how many times data will be written
template <class T>
void OutConnector<T>::writeData(const T *data)
{       
    try {
        // if m_index equals CIRCULAR_SIZE, zero m_index
        if(m_index == CIRCULAR_SIZE)
            m_index = 0;
        
        // clear key
        memset(m_key, KEY_SIZE, '\0');
        // fill key with index
        sprintf(m_key,"%d", m_index++ % CIRCULAR_SIZE);
        
        // if key is already in memory, destroy it
        if(m_msm.find<T>(m_key).first)
            m_msm.destroy<T>(m_key);

        // construct data with key
        m_msm.construct<T>(m_key)(*data);
    } 
    catch(boost::interprocess::bad_alloc &ex) {
        std::cerr << "ERROR : " << ex.what() << std::endl;
        exit(EXIT_FAILURE);
    }
    catch(boost::interprocess::interprocess_exception &ex) {
        std::cerr << "ERROR : " << ex.what() << std::endl;        
         exit(EXIT_FAILURE);
    }
    
    return;
}

// function that gets connected in connectors count
template <class T>
int OutConnector<T>::getConnectedConnectorsCount() const
{
    return m_in_connectors_count;
}

// function that generates unique name for managed shared memory
template <class T>
void OutConnector<T>::generateName()
{
	std::stringstream ss;

    // create an unique id with random_generator
	ss << boost::uuids::random_generator()();
    // copy generated unique id to m_msm_name
	strcpy(m_msm_name, ss.str().c_str());
    
    return;
}

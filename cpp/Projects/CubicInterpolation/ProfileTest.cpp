#include "Profile.hpp"
#include <unistd.h>
#include <tbb/concurrent_queue.h>
#include <pthread.h>
#include <stdio.h>

tbb::concurrent_queue<float> msg_queue;
pthread_t sampler_thread;
pthread_t printer_thread;
pthread_mutex_t printer_mutex;
pthread_cond_t printer_condvar;
Profile profile;

// printer thread 
void *printer(void *param)
{
  float res = 0.0;

  while(true) {
    // wait condvar
    pthread_cond_wait(&m_profiler_condvar, &m_profiler_mutex);
    
    if(!m_queue.empty()) {
      // pop value from queue
      m_queue.try_pop(res);
      // write file
      std::cout << res << std::endl;
    }
  }

  // exit from thread
  pthread_exit(NULL);

  return NULL;
}

// timer thread
void *sampler(void *param)
{
  float t = 1.0;
  float val;
  struct timeval tv;

  while(true) {

    // Wait for timer event

    gettimeofday(&tv, NULL);
    std::cout << "[" << std::setw(5) << (tv.tv_sec * 1000 + tv.tv_usec / 1000) % 100000 << "] ";

    val = profile.value(param);
    m_queue.push(val);
    pthread_cond_signal(&m_profiler_condvar);

    // increase second by 0.1 second or 100 milliseconds
    t += 0.1;
  }

  return NULL;
}

// main function
int main()
{
  std::cout.unsetf (std::ios::floatfield);
  std::cout.precision(3);

  // load coefficients
  profile.load("coefficients.dat");

  // start timer and profiler threads
  pthread_create(&m_sampler_thread, NULL, sampler, NULL);
  pthread_create(&m_printer_thread, NULL, printer, NULL);

  // join threads when terminate
  pthread_join(m_timer_thread, NULL);
  pthread_join(m_profiler_thread, NULL);

  return 0;
}

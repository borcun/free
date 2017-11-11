#include "Interpolation.hpp"

  // input file
  FILE *m_ifile;
  // output file
  FILE *m_ofile;
  // input file path
  char *m_ipath;
  // output file path
  char *m_opath;
  // estimation thread
  pthread_t m_est_thread;
  // mutex
  pthread_mutex_t m_est_mutex;
  // condition variable
  pthread_cond_t m_est_cvar;
  // condition state
  bool m_est_cstat;
  // concurrent queue
  tbb::concurrent_queue<float> m_queue;

void *estimate(void *param)
{
  FILE *fptr = (FILE *)param;
  char coeff[MAX_CHAR];
  int t = 1;

  while(NULL != fgets(coeff, MAX_CHAR, fptr)) {
    float a = atof(strtok(coeff, ","));
    float b = atof(strtok(NULL, ","));
    float c = atof(strtok(NULL, ","));
    float d = atof(strtok(NULL, ","));

    for(int i=1 ; i < 10 ; ++i) {
      m_queue.push(a * pow(t + i * 0.1, 3) +
		   b * pow(t + i * 0.1, 2) +
		   c * (t + i * 0.1) +
		   d);
    }

    // increase time
    ++t;
  }

  // set cstat as false
  m_est_cstat = false;
  // wait signal
  pthread_cond_wait(&m_est_cvar, &m_est_mutex);

  // exit from thread
  pthread_exit(NULL);

  return NULL;
}

void printResult()
{
  float res = 0.0;

  // get result from queue
  m_queue.try_pop(res);

  fprintf(m_ofile, "%f\n", res);
}

// default constructor
Interpolation::Interpolation()
{
}

// constructor
Interpolation::Interpolation(const char *ipath, const char *opath)
{
  setPaths(ipath, opath);
}

Interpolation::~Interpolation()
{
  delete m_ipath;
  delete m_opath;
}

void Interpolation::setPaths(const char *ipath, const char *opath)
{
  m_ipath = new char[ strlen(ipath) ];
  m_opath = new char[ strlen(opath) ];

  strcpy(m_ipath, ipath);
  strcpy(m_opath, opath);

  return;
}

void Interpolation::interpolate()
{
  if(NULL == (m_ifile = fopen(m_ipath, "r"))) {
      std::cerr << "Input file can not be opened" << std::endl;
      return;
  }

  if(NULL == (m_ofile = fopen(m_opath, "w+"))) {
      std::cerr << "Output file can not be opened" << std::endl;
      return;
  }

  // run thread
  pthread_create(&m_est_thread, NULL, estimate, (void *)m_ifile);

  // run timer
  timer(1000000, printResult);

  // join thread
  pthread_join(m_est_thread, NULL);

  // close input file
  fclose(m_ifile);
  // close output file
  fclose(m_ofile);
}

void Interpolation::timer(const int microsec, void(*callback)())
{
  while(m_est_cstat) {
    callback();
    usleep(microsec);
  }

  pthread_cond_signal(&m_est_cvar);
  return;
}

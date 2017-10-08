#include <iostream>
#include <Configuration.h>
#include <RBDL_Sim_Configuration.h>
#include <Utils/utilities.hpp>

#include <math.h>
#include <chrono>
#include <vector>
#include <unistd.h>


int main(int argc, char ** argv){

  DynControlTester * ctrl_tester = new DynCtrlTester();
  ctrl_tester->Initialization();

  while(true){
    ctrl_tester->OneStepTest();
    usleep(ctrl_tester->m_usleep_time);
  }
  return 0;
}

////////////////////////////////////////////////////////
//             Computation time checker               //
////////////////////////////////////////////////////////
#ifdef CHECK_COMPUTATION_TIME

//////// Get Command
std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();

///// Place To Put the Functions

std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
std::chrono::duration<double> time_span1 = std::chrono::duration_cast< std::chrono::duration<double> >(t2 - t1);
std::cout << "[Plotter] Time to compute: " << time_span1.count()*1000. << " (ms)."<<std::endl;;
//////// End of Get Command

#endif

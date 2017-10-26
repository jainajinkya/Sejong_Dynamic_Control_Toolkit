#include "Mercury_Dyn_environment.hpp"
#include "common/utils.h"
#include <Utils/wrap_eigen.hpp>
#include <Utils/utilities.hpp>

// #define Measure_Time
#ifdef Measure_Time
#include <chrono>
using namespace std::chrono;
#endif

// #define SENSOR_NOISE
#define SENSOR_DELAY 0 // Sensor_delay* SERVO_RATE (sec) = time delay 


Mercury_Dyn_environment::Mercury_Dyn_environment(){
  m_Mercury = new Mercury(Vec3(0.0, 0.0, 0.0), srSystem::FIXED, srJoint::TORQUE);
  m_Space = new srSpace();
  m_ground = new Ground();

  m_Space->AddSystem(m_ground->BuildGround());
  m_Space->AddSystem((srSystem*)m_Mercury);
  m_Space->DYN_MODE_PRESTEP();

  m_Space->SET_USER_CONTROL_FUNCTION_2(ContolFunction);
  m_Space->SetTimestep(SERVO_RATE);
  m_Space->SetGravity(0.0,0.0,-9.8);
  m_Space->SetNumberofSubstepForRendering(6);
}

void Mercury_Dyn_environment::ContolFunction( void* _data ) {
  static int iter(0);
  ++iter;
}


void Mercury_Dyn_environment::Rendering_Fnc(){}

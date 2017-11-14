#ifndef		MERCURY_DYN_ENVIRONMENT
#define		MERCURY_DYN_ENVIRONMENT

#include "Mercury.hpp"
#include "srDyn/srSystem.h"
#include "srDyn/srCollision.h"
#include "Ground.h"
#include <vector>
#include "Configuration.h"

class interface;

class Mercury_Dyn_environment
{
public:
  Mercury_Dyn_environment();
  ~Mercury_Dyn_environment();

  static void ContolFunction(void* _data);

  void Rendering_Fnc();
  void _FixXY();
  
  interface* interface_;
  Mercury*	m_Mercury;
  srSpace*	m_Space;
  Ground*	m_ground;

  void getIMU_Data(std::vector<double> & imu_acc,
                   std::vector<double> & imu_ang_vel);
private:
  void _ParamterSetup();
  int num_substep_rendering_;
  double release_time_;
};

#endif

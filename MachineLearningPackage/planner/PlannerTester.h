#ifndef PLANNER_TESTER
#define PLANNER_TESTER

#include <Utils/wrap_eigen.hpp>

#include <LocomotionPlanner/EnvironmentSetup/FlatTerrain.h>
#include <LocomotionPlanner/EnvironmentSetup/Const_Height_Surface.h>
#include <LocomotionPlanner/PIPM_Planner_Analytic.h>


class LIPM_3D;

class PlannerTester{
public:
  PlannerTester (const sejong::Vector & ini, LIPM_3D* env);
  ~PlannerTester(){}

  void check();
protected:
  int num_step_;
  void _TestFunction();
  void _SetParam();
  // Set Followings
  Planner* planner_;
  PlanningParam* param_;
  Terrain* terrain_;
  CoMSurface* com_surf_;

  sejong::Vector ini_state_;
  LIPM_3D * pipm_env_;
};

#endif

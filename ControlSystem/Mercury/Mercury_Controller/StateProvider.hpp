#ifndef STATE_PROVIDER_MERCURY
#define STATE_PROVIDER_MERCURY

#include <Utils/utilities.hpp>
#include <Utils/wrap_eigen.hpp>
#include <Configuration.h>

using namespace sejong;

class StateProvider{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  static StateProvider* GetStateProvider();
  ~StateProvider(){}

  sejong::Quaternion body_ori_;
  sejong::Vect3 body_ang_vel_;
  sejong::Vect3 imu_acc_inc_;
  sejong::Vect3 imu_ang_vel_;
  sejong::Vect3 imu_acc_;
  // Important!!!!!!!!
  int stance_foot_;

  bool initialized_;
  double curr_time_;
  int system_count_;

  Vector Q_;
  Vector Qdot_;
  Vector curr_torque_;

  sejong::Vector reaction_forces_;

  sejong::Vect3 global_pos_local_;
  sejong::Vect2 des_location_;

  sejong::Vect3 Rfoot_pos_;
  sejong::Vect3 Lfoot_pos_;
  sejong::Vect3 Rfoot_vel_;
  sejong::Vect3 Lfoot_vel_;

  sejong::Vect3 CoM_pos_;
  sejong::Vect3 CoM_vel_;

  void SaveCurrentData();

private:
  StateProvider();
};


#endif

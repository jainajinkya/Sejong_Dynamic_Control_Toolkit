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
  // Important!!!!!!!!
  int stance_foot_;

  bool initialized_;
  double curr_time_;
  int system_count_;

  Vector Q_;
  Vector Qdot_;
  Vector curr_torque_;

  sejong::Vector reaction_forces_;

  ///// Desired
  sejong::Vect3 Body_pos_des_;
  sejong::Vect3 Body_vel_des_;
  sejong::Vect3 Body_acc_des_;

  ////// Current
  sejong::Vect3 Body_pos_;
  sejong::Vect3 Body_vel_;

private:
  StateProvider();
};


#endif

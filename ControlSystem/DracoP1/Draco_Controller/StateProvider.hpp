#ifndef STATE_PROVIDER_H
#define STATE_PROVIDER_H

#include <Utils/utilities.hpp>
#include <Utils/wrap_eigen.hpp>
#include <Configuration.h>

using namespace sejong;

class StateProvider{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  static StateProvider* GetStateProvider();

  ~StateProvider(){}

  void SaveSingleReactionForce(const sejong::Vector & Fr);
  void Push_PathNode(const sejong::Vector & node);

  bool initialized_;
  double curr_time_;
  int system_count_;

  Vector Q_;
  Vector Qdot_;
  Vector curr_torque_;

  ///// Desired
  sejong::Vect3 CoM_pos_des_;
  sejong::Vect3 CoM_vel_des_;
  sejong::Vect3 CoM_acc_des_;

  ////// Current
  sejong::Vect3 CoM_pos_;
  sejong::Vect3 CoM_vel_;

private:
  StateProvider();
};


#endif

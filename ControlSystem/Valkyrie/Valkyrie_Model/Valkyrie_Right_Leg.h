#ifndef VALKYRIE_RIGHT_LEG_MODEL
#define VALKYRIE_RIGHT_LEG_MODEL

#include <rbdl/rbdl.h>
#include <Utils/wrap_eigen.hpp>

class Valkyrie_Right_Leg{
 public:
  static Valkyrie_Right_Leg* GetValkyrieRightLeg();
  ~Valkyrie_Right_Leg();

  void UpdateKinematics(const sejong::Vector & Q, const sejong::Vector & Qdot );
  void getRightFootPosition(sejong::Vect3 & lfoot_pos);
  void getRightFootVelocity(sejong::Vect3 & lfoot_vel);
  void getRightFootAngVel(sejong::Vect3 & ang_vel);
  void getRightFootOrientation(sejong::Quaternion & ori);

 private:
  int bodyid_;
  RigidBodyDynamics::Model* model_;
  Valkyrie_Right_Leg();
};
#endif

#ifndef VALKYRIE_LEFT_LEG_MODEL
#define VALKYRIE_LEFT_LEG_MODEL

#include <rbdl/rbdl.h>
#include <Utils/wrap_eigen.hpp>

class Valkyrie_Left_Leg{
 public:
  static Valkyrie_Left_Leg* GetValkyrieLeftLeg();
  ~Valkyrie_Left_Leg();

  void UpdateKinematics(const sejong::Vector & Q, const sejong::Vector & Qdot );
  void getLeftFootPosition(sejong::Vect3 & lfoot_pos);
  void getLeftFootVelocity(sejong::Vect3 & lfoot_vel);
  void getLeftFootAngVel(sejong::Vect3 & ang_vel);
  void getLeftFootOrientation(sejong::Quaternion & ori);

 private:
  int bodyid_;
  RigidBodyDynamics::Model* model_;
  Valkyrie_Left_Leg();
};
#endif

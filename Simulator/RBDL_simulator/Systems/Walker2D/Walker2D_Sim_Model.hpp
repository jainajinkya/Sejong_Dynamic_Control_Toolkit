#ifndef WALKER_2D_RBDL_SIMULATION_MODEL
#define WALKER_2D_RBDL_SIMULATION_MODEL

#include <rbdl/rbdl.h>
#include <Utils/wrap_eigen.hpp>

using namespace sejong;

class Walker2D_Sim_Model{
public:
  // EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  static Walker2D_Sim_Model* GetWalker2D_Sim_Model();
  virtual ~Walker2D_Sim_Model(void);

  bool getMassInertia(sejong::Matrix & A);
  bool getGravity(Vector & grav) ;
  bool getCoriolis(Vector & coriolis) ;

  // (x, z, Ry)
  void getPos(int link_id, Vect3 & pos);
  void getVel(int link_id, Vect3 & vel) ;
  void getFullJacobian(int link_id, sejong::Matrix & J) const ;
  void getFullJacobianDot(int link_id, sejong::Matrix & Jdot) const ;

  void UpdateModel(const sejong::Vector & q, const sejong::Vector & qdot);
protected:
  Matrix A_;
  Vector grav_;
  Vector coriolis_;

  RigidBodyDynamics::Model* model_;
  unsigned int _find_body_idx(int id) const ;

private:
  Walker2D_Sim_Model();
};

#endif

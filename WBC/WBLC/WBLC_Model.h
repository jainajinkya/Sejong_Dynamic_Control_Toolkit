#ifndef WBLC_MODEL_H
#define WBLC_MODEL_H

#include <Utils/wrap_eigen.hpp>

class WBLC_Model{
 public:
  WBLC_Model(){}
  virtual ~WBLC_Model() {}

  virtual bool getInverseMassInertia(sejong::Matrix & Ainv) = 0;
  virtual bool getCoriolis(sejong::Vector & coriolis) = 0;
  virtual bool getGravity(sejong::Vector & grav) = 0;

  virtual void getCentroidJacobian(sejong::Matrix & Jcm) = 0;
  virtual void getCentroidInertia(sejong::Matrix & Ig) = 0;
  virtual void getPosition(const sejong::Vector & q,
                           int link_id, sejong::Vect3 & pos) = 0;
  virtual void getFullJacobian(const sejong::Vector & q, int link_id, sejong::Matrix & J) const = 0;

  virtual void getCoMPosition(const sejong::Vector & q, sejong::Vect3 & com_pos, bool update = false) = 0;
  virtual void getCoMVelocity(const sejong::Vector & q, const sejong::Vector & qdot, sejong::Vect3 & com_vel) = 0;
};

#endif

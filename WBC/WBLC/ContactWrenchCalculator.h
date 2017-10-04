#ifndef CONTACT_WRENCH_CALCULATOR_H
#define CONTACT_WRENCH_CALCULATOR_H

#include "WBLC_Model.h"
#include <Utils/wrap_eigen.hpp>
#include <Utils/utilities.h>
#include <Contact.h>

class ContactWrenchCalculator{
public:
  ContactWrenchCalculator(WBLC_Model*, ContactHandler*);
  virtual ~ContactWrenchCalculator();

  virtual void PrintName();
  bool GetReactionForce(sejong::Vector & Fr, bool update = true);

  void getCentroidCtrl_Jacobian(sejong::Matrix & Jcm){
    robot_model_->getCentroidJacobian(Jcm);
  }

  void SetCentroidConfiguration(const sejong::Vector & cent_conf,
                                const sejong::Vector & cent_vel);

  void SetCentroidConfiguration(const sejong::Vector & des_ff,
                                const sejong::Vector & cent_conf,
                                const sejong::Vector & cent_vel);

  // Linear
  void getCoMAcc(sejong::Vector & com_acc){ com_acc = CoM_acc_; }
  // Angular (Centroidal Angular Acceleration)
  void getCAMAcc(sejong::Vector & cam_acc){ cam_acc = CAM_acc_; }

  void setCAM_Acceleration(const sejong::Vector & cm_ang_acc){ CAM_acc_ = cm_ang_acc; }

  void getContactJacobian(sejong::Matrix & Jc){ Jc = Jc_; }
  void getCentroidInertia(sejong::Matrix & Ig){
    robot_model_->getCentroidInertia(Ig);
  }
  void getW(sejong::Matrix & W){ W = W_;}
  void getCentroidData(sejong::Vector & cm_pos,
                       sejong::Vector & cm_vel,
                       sejong::Vector & cm_pos_des,
                       sejong::Vector & cm_vel_des,
                       sejong::Vector & cm_acc_des);

  virtual void PrepareOptimization() = 0;

  virtual void GetByProduct(sejong::Vector & ret, sejong::Matrix & ret_mt){}

  sejong::Vector Kp_;
  sejong::Vector Kv_;

protected:
  WBLC_Model* robot_model_;
  ContactHandler* contact_handler_;
  virtual bool _CalculateReactionForce(sejong::Vector & Fr) = 0;
  virtual void _SetMatrixW() = 0;
  virtual void _SetContactJacobian() = 0;

  void _PreProcessing();
  void _PostProcessing();
  bool b_setCoM_;
  bool b_prepared_;

  sejong::Matrix Jc_;
  sejong::Vector des_ff_;
  sejong::Vector des_conf_;
  sejong::Vector des_vel_;

  sejong::Vector cm_acc_;
  sejong::Vector CoM_acc_;
  sejong::Vector CAM_acc_;

  sejong::Vector cm_pos_;
  sejong::Vector cm_vel_;
  sejong::Vector cm_ang_;
  sejong::Matrix W_;
  sejong::Vector Fr_;
};
#endif

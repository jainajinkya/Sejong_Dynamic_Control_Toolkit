#ifndef WHOLE_BODY_DYNAMIC_CONTROLLER_DRACO_P1_ROTATION
#define WHOLE_BODY_DYNAMIC_CONTROLLER_DRACO_P1_ROTATION

#include <DracoP1Rot_Controller/DracoController.hpp>
#include <WBDC/WBDC.h>
#include <WBDC/WBDC_2D.h>
#include <WBDC/WBDC_new.h>
#include <WBDC/WBDC_2D_vir_acc.h>

#define DIM_ReactionForce 3 // foot (x, z, ry)
/* #define DIM_WBDC_TASK 6 // foot(x,z,ry), joint (q0, q1, q2) */
#define DIM_WBDC_TASK 5 // foot(x,z,ry), hip (x, z)
#define DIM_ADJUSTABLE_TASK 3
class WBDC_Ctrl: public DracoController{
public:
  WBDC_Ctrl();
  virtual ~WBDC_Ctrl();

  virtual void ComputeTorqueCommand(sejong::Vector & gamma);
  virtual void Initialization();

protected:
  // Data Save
  sejong::Vector des_pos_;
  sejong::Vector act_pos_;
  sejong::Vector act_vel_;
  sejong::Vector xddot_cmd_;
  Eigen::Matrix<double, DIM_ReactionForce + DIM_ADJUSTABLE_TASK + NUM_VIRTUAL, 1> optimization_results_;


  // dim rf, dim cm, dim seleted task, dim task
  WBDC<DIM_ReactionForce, 3, DIM_ADJUSTABLE_TASK, DIM_WBDC_TASK> wbdc_;
  WBDC_2D<DIM_ReactionForce, DIM_ADJUSTABLE_TASK, DIM_WBDC_TASK> wbdc_2d_;
  WBDC_new<DIM_ReactionForce, DIM_ADJUSTABLE_TASK, DIM_WBDC_TASK> wbdc_new_;
  WBDC_2D_vir_acc<DIM_ReactionForce, DIM_ADJUSTABLE_TASK, DIM_WBDC_TASK> wbdc_2d_vir_acc_;

  void _jpos_ctrl(sejong::Vector & gamma);
  void _body_ctrl(sejong::Vector & gamma);
  void _body_ctrl2(sejong::Vector & gamma);
  int count_command_;

  sejong::Vect3 com_init_;
  sejong::Vect3 body_init_;
  sejong::Vector jpos_ini_;
  sejong::Vector jpos_des_;
  sejong::Vect3 foot_ini_;
  sejong::Vect2 des_gap_;

  SJLinkID link_id_;
  int initialization_count_;

  Eigen::Matrix<double, NUM_ACT_JOINT, 1> tau_min_;
  Eigen::Matrix<double, NUM_ACT_JOINT, 1> tau_max_;
  Eigen::Matrix<double, NUM_ACT_JOINT, 1> cmd_;

  void _CopyMassGravityCoriolis();
  void _CentroidMatrixSetting();
  Eigen::Matrix<double, NUM_QDOT, NUM_QDOT> A_fix_;
  Eigen::Matrix<double, NUM_QDOT, 1> cori_fix_;
  Eigen::Matrix<double, NUM_QDOT, 1> grav_fix_;

  Eigen::Matrix<double, 3, 3> Icm_;
  Eigen::Matrix<double, 3, NUM_QDOT> Jcm_;
  Eigen::Matrix<double, 3, 1> JcmDotQdot_;
};


#endif

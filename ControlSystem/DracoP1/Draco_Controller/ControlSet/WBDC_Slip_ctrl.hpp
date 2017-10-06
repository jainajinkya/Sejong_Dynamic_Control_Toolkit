#ifndef WHOLE_BODY_DYNAMIC_CONTROLLER_SLIP
#define WHOLE_BODY_DYNAMIC_CONTROLLER_SLIP

#include <Draco_Controller/DracoController.hpp>
#include <WBDC/WBDC_Slip.h>
#include <WBDC/WBDC_integer.h>

#define DIM_RForce 3 // foot (x, z, ry)
#define DIM_NONSLIP_TASK 2 // foot(z,ry)
#define DIM_MOTION_TASK 2 //hip (x, z)
#define DIM_SLIP_TASK 1   //foot (x)

class WBDC_Slip_ctrl: public DracoController{
public:
  WBDC_Slip_ctrl();
  virtual ~WBDC_Slip_ctrl();

  virtual void ComputeTorqueCommand(sejong::Vector & gamma);
  virtual void Initialization();

protected:
  // Data Save
  sejong::Vector des_pos_;
  sejong::Vector act_pos_;
  sejong::Vector act_vel_;
  sejong::Vector xddot_cmd_;
  Eigen::Matrix<double, DIM_RForce + DIM_MOTION_TASK + 2*DIM_SLIP_TASK, 1> optimization_results_;

  void _body_ctrl(sejong::Vector & gamma);

  int count_command_;
  double state_machine_time_;
  double start_time_;

  sejong::Vect3 body_init_;
  sejong::Vect3 foot_ini_;
  sejong::Vect2 des_gap_;

  int initialization_count_;
  WBDC_Slip<DIM_RForce, DIM_NONSLIP_TASK, DIM_MOTION_TASK, DIM_SLIP_TASK> wbdc_slip_;
  WBDC_integer<DIM_RForce, DIM_NONSLIP_TASK, DIM_MOTION_TASK, DIM_SLIP_TASK> wbdc_integer_;

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

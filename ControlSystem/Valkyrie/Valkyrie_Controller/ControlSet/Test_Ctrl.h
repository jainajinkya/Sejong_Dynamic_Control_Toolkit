#ifndef CONTROLLER_TEST
#define CONTROLLER_TEST

#include <Valkyrie_Controller/ValkyrieController.h>
#include <WBDC/WBDC_3D.h>

/* #define TIME_MEASUREMENT_TEST_CTRL */
/* #define PRINT_OUT_MESSAGE */
#define TIME_MEASUREMENT_TEST_BC

#define DIM_RForce 12
#define DIM_CM 6
#define DIM_BODY_ORI 3
#define DIM_PELVIS_ORI 3
#define DIM_RIGHT_HAND_POS 3
#define DIM_LEFT_HAND_POS 3


#define DIM_RelaxedTask 0

// JPos Task
#define DIM_JPOS_TASK (DIM_RForce + NUM_ACT_JOINT)

// Multiple Tasks 1 (CM)
#define DIM_MULTI_TASK (DIM_RForce + DIM_CM + NUM_ACT_JOINT)

// Multiple Tasks 2 (right hand)
#define DIM_MULTI_TASK2 (DIM_RForce + DIM_CM + DIM_RIGHT_HAND_POS + NUM_ACT_JOINT)

// Multiple Tasks 3 (Both hands)
#define DIM_MULTI_TASK3 (DIM_RForce + DIM_CM + DIM_RIGHT_HAND_POS + DIM_LEFT_HAND_POS + NUM_ACT_JOINT)

// ******************************************* //
// Change whenever task types are changed.
/* #define DIM_TASK_GLOBAL DIM_JPOS_TASK */
/* #define DIM_TASK_GLOBAL DIM_MULTI_TASK */
/* #define DIM_TASK_GLOBAL DIM_MULTI_TASK2 */
#define DIM_TASK_GLOBAL DIM_MULTI_TASK3

class Test_Ctrl: public ValkyrieController{
public:
  Test_Ctrl();
  virtual ~Test_Ctrl();

  virtual void ComputeTorqueCommand(sejong::Vector & gamma);
  virtual void Initialization();

  sejong::Vector jpos_ini_;
  sejong::Vector jpos_des_;
  sejong::Vector jvel_des_;

protected:
  void _PrepareMatriceVectors();
  void _GenerateBC();
  void _GenerateBC_MultiTask();
  void _GenerateBC_MultiTask2();
  void _GenerateBC_MultiTask_BothHand();

  void _SetJointTaskCMD_Multi(const int st_idx, Eigen::Matrix<double, DIM_TASK_GLOBAL, 1> & x_task);
  void _SetCMTaskCMD_Multi(const int st_idx, Eigen::Matrix<double, DIM_TASK_GLOBAL, 1> & x_task);
  void _SetRightHandPosTaskCMD(const int st_idx, Eigen::Matrix<double, DIM_TASK_GLOBAL, 1> & x_task);
  void _SetLeftHandPosTaskCMD(const int st_idx, Eigen::Matrix<double, DIM_TASK_GLOBAL, 1> & x_task);

  void _GenerateSf();

  // DIM_RF, DIM_selected task, Dim_task
  WBDC_3D<DIM_RForce, DIM_RelaxedTask, DIM_TASK_GLOBAL> wbdc_;

  sejong::Vect3 com_init_;
  sejong::Quaternion body_ori_ini_;
  sejong::Quaternion pelvis_ori_ini_;
  sejong::Vect3 rhand_init_;
  sejong::Vect3 lhand_init_;

  double start_time_;
  double state_machine_time_;
  int count_command_;

  // Function
  void _jpos_ctrl(sejong::Vector & gamma);
  void _multi_task_ctrl(sejong::Vector & gamma);
  void _multi_task_ctrl2(sejong::Vector & gamma);
  void _multi_task_ctrl_both_hand(sejong::Vector & gamma);

  // U
  Eigen::Matrix<double, 16, 6> Uc_; // Contact U*R*Fr < 0
  Eigen::Matrix<double, 16, 6> UcR_right_; // Contact U*R*Fr_right < 0
  Eigen::Matrix<double, 16, 6> UcR_left_; // Contact U*R*Fr_left < 0

  // B, c
  Eigen::Matrix<double, NUM_QDOT, DIM_TASK_GLOBAL> B_;
  Eigen::Matrix<double, NUM_QDOT, 1> c_;

  // A, b, g, Jc
  Eigen::Matrix<double, NUM_QDOT, NUM_QDOT> A_fixed_;
  Eigen::Matrix<double, NUM_QDOT, 1> grav_fixed_;
  Eigen::Matrix<double, NUM_QDOT, 1> cori_fixed_;
  Eigen::Matrix<double, DIM_RForce, NUM_QDOT> Jc_; // Right & Left
  sejong::Matrix Jcdot_; // Right & Left

  // Centroidal Momentum
  Eigen::Matrix<double, 6, 6> Icm_; // Angular & Linear
  Eigen::Matrix<double, 6, NUM_QDOT> Jcm_;
  Eigen::Matrix<double, 6, 1> gcm_;
  Eigen::Matrix<double, 6, 1> JcmDotQdot_;

  Eigen::Matrix<double, 6, DIM_RForce> Sf_;
  Eigen::Matrix<double, NUM_ACT_JOINT, 1> tau_max_;
  Eigen::Matrix<double, NUM_ACT_JOINT, 1> tau_min_;
  Eigen::Matrix<double, NUM_ACT_JOINT, 1> cmd_;

  Eigen::Matrix<int, DIM_TASK_GLOBAL, 1> selected_task_;

  sejong::Vect3 com_pos_des_;
  sejong::Vect3 com_vel_des_;

  sejong::Vect3 com_pos_;
  sejong::Vect3 com_vel_;
  sejong::Vect3 cam_vel_;

  sejong::Vect3 lfoot_pos_;
  sejong::Vect3 rfoot_pos_;

  sejong::Vect3 lhand_pos_;
  sejong::Vect3 rhand_pos_;

  sejong::Vect3 lhand_vel_;
  sejong::Vect3 rhand_vel_;

  sejong::Vect3 des_lhand_pos_;
  sejong::Vect3 des_rhand_pos_;
  sejong::Vect3 des_lhand_vel_;
  sejong::Vect3 des_rhand_vel_;


  sejong::Vector des_pos_;
  sejong::Vector act_pos_;
  sejong::Vector act_vel_;
  sejong::Vector xddot_cmd_;
  Eigen::Matrix<double, DIM_RForce + DIM_RelaxedTask + NUM_VIRTUAL, 1> optimization_results_;

  SJLinkID link_id_;
  int initialization_count_;
};


#endif

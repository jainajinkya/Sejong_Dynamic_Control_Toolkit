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

  void SaveDoubleReactionForce(const sejong::Vector & Fr);
  void SaveRightSingleReactionForce(const sejong::Vector & Fr);
  void SaveLeftSingleReactionForce(const sejong::Vector & Fr);
  void SaveStatEqConvexHull(const sejong::Vect3 & point);
  void Save3DCoMAccConvexHull(const sejong::Vect3 & point);
  void SaveCoMPos(const sejong::Vect3 & point);
  void SaveCoMAcc(const sejong::Vector & acc);

  void Push_PathNode(const sejong::Vector & node);
  void CleaningList();
  SJLinkID GetStanceFoot(){ return stance_foot_; }
  void SetStanceFoot(const SJLinkID & stance_foot){ stance_foot_ = stance_foot; }

  SJLinkID stance_foot_;
  sejong::Vector global_stance_foot_pos_;

  bool initialized_;
  double curr_time_;
  int system_count_;

  Vector Q_;
  Vector Qdot_;
  Vector curr_torque_;

  // The list we are going to save
  Vector Fr_;
  Vector cent_ang_vel_;
  ///// Desired
  // Body & Pelvis Orientation
  sejong::Quaternion pelvis_ori_des_;
  sejong::Vector pelvis_ang_vel_des_;
  sejong::Quaternion body_ori_des_;
  sejong::Vector body_ang_vel_des_;

  sejong::Vect2 Global_CoM_pos_des_;
  sejong::Vect2 Global_CoM_pos_;

  sejong::Vect3 CoM_pos_des_;
  sejong::Vect3 CoM_vel_des_;
  sejong::Vect3 CoM_acc_des_;

  sejong::Vect3 Body_pos_des_;
  sejong::Vect3 Body_vel_des_;
  sejong::Vect3 Body_acc_des_;

  // Right Foot
  sejong::Vect3 rfoot_pos_des_;
  sejong::Vect3 rfoot_vel_des_;
  sejong::Vect3 rfoot_acc_des_;
  sejong::Quaternion rfoot_ori_des_;
  sejong::Vect3 rfoot_ang_vel_des_;

  // Left Foot
  sejong::Vect3 lfoot_pos_des_;
  sejong::Vect3 lfoot_vel_des_;
  sejong::Vect3 lfoot_acc_des_;
  sejong::Quaternion lfoot_ori_des_;
  sejong::Vect3 lfoot_ang_vel_des_;

  // Stance Foot
  sejong::Vect3 stance_foot_vel_;
  
  ////// Current
  sejong::Vect3 Body_pos_;
  sejong::Vect3 Body_vel_;
  sejong::Vect3 Body_acc_;

  // Body & Pelvis Orientation
  sejong::Quaternion pelvis_ori_;
  sejong::Vector pelvis_ang_vel_;
  sejong::Quaternion body_ori_;
  sejong::Vector body_ang_vel_;

  sejong::Vect3 CoM_pos_;
  sejong::Vect3 CoM_vel_;
  // Right Foot
  sejong::Vect3 rfoot_pos_;
  sejong::Vect3 rfoot_vel_;
  sejong::Quaternion rfoot_ori_;
  sejong::Vect3 rfoot_ang_vel_;
  // Left Foot
  sejong::Vect3 lfoot_pos_;
  sejong::Vect3 lfoot_vel_;
  sejong::Quaternion lfoot_ori_;
  sejong::Vect3 lfoot_ang_vel_;

  // Local Frame
  sejong::Vect2 global_pos_local_frame_;

  // For srLib Drawing
  std::vector<sejong::Vector> path_;
  std::vector<sejong::Vector> foot_placement_list_;
  std::vector<sejong::Quaternion> foot_orientation_list_;
  std::vector<sejong::Vect3> stat_eq_convex_hull;
  std::vector<sejong::Vect3> comacc_convex_hull;
  sejong::Vector CoM_acc;

  bool b_lcontact_;
  bool b_rcontact_;
  bool b_both_contact_;
private:
  StateProvider();
};


#endif

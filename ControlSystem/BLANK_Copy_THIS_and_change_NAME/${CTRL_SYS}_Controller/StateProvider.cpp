#include "StateProvider.h"
#include <Utils/DataManager.h>

#define TASK_LEN 10

StateProvider* StateProvider::GetStateProvider(){
    static StateProvider state_provider_;
    return &state_provider_;
}

StateProvider::StateProvider(): initialized_(false),
                                system_count_(0),
                                b_both_contact_(true),
                                // Arbitrary Large number
                                Q_(NUM_Q),
                                Qdot_(NUM_QDOT),
                                Fr_(24),
                                pelvis_ang_vel_des_(3),
                                body_ang_vel_des_(3),
                                pelvis_ang_vel_(3),
                                body_ang_vel_(3),
                                rfoot_pos_des_(3),
                                rfoot_vel_des_(3),
                                rfoot_acc_des_(3),
                                rfoot_ang_vel_des_(3),
                                lfoot_pos_des_(3),
                                lfoot_vel_des_(3),
                                lfoot_acc_des_(3),
                                lfoot_ang_vel_des_(3),
                                rfoot_pos_(3),
                                rfoot_vel_(3),
                                rfoot_ang_vel_(3),
                                lfoot_pos_(3),
                                lfoot_vel_(3),
                                lfoot_ang_vel_(3),
                                cent_ang_vel_(3),
                                // stance_foot_(LK_leftFoot),
                                stance_foot_(LK_leftCOP_Frame),
                                global_stance_foot_pos_(3)
{
  path_.clear();
  Fr_.setZero();
  Q_.setZero();
  Qdot_.setZero();
  Q_[NUM_QDOT] = 1.0;
  global_pos_local_frame_.setZero();

  rfoot_pos_des_.setZero();
  rfoot_vel_des_.setZero();
  rfoot_pos_.setZero();
  rfoot_vel_.setZero();

  rfoot_ang_vel_des_.setZero();
  rfoot_ang_vel_.setZero();

  lfoot_pos_des_.setZero();
  lfoot_vel_des_.setZero();
  lfoot_pos_.setZero();
  lfoot_vel_.setZero();

  lfoot_ang_vel_des_.setZero();
  lfoot_ang_vel_.setZero();

  Global_CoM_pos_des_.setZero();
  Global_CoM_pos_.setZero();

  DataManager* data_manager = DataManager::GetDataManager();

  data_manager->RegisterData(&curr_time_, DOUBLE, "time");
  data_manager->RegisterData(&Q_, SJ_VEC, "config", NUM_Q);
  data_manager->RegisterData(&Qdot_, SJ_VEC, "qdot", NUM_QDOT);
  // data_manager->RegisterData(&global_pos_local_frame_, VECT2, "global_offset", 2);
  // CoM
  data_manager->RegisterData(&Global_CoM_pos_, VECT2, "global_com_pos", 2);
  data_manager->RegisterData(&Global_CoM_pos_des_,  VECT2, "global_com_pos_des", 2);

  data_manager->RegisterData(&CoM_pos_, VECT3, "com_pos", 3);
  data_manager->RegisterData(&CoM_vel_, VECT3, "com_vel", 3);
  data_manager->RegisterData(&CoM_pos_des_, VECT3, "com_pos_des", 3);
  data_manager->RegisterData(&CoM_vel_des_, VECT3, "com_vel_des", 3);
  data_manager->RegisterData(&CoM_acc_des_, VECT3, "com_acc_des", 3);

  // [Pelvis & Body Orientation] Desired
  data_manager->RegisterData(&pelvis_ori_des_, QUATERNION, "pelvis_ori_des", 4);
  data_manager->RegisterData(&body_ori_des_, QUATERNION, "body_ori_des", 4);
  data_manager->RegisterData(&pelvis_ori_, QUATERNION, "pelvis_ori", 4);
  data_manager->RegisterData(&body_ori_, QUATERNION, "body_ori", 4);
  // [Pelvis & Body Orientation] Actual
  data_manager->RegisterData(&pelvis_ang_vel_des_, SJ_VEC, "pelvis_ang_vel_des", 3);
  data_manager->RegisterData(&body_ang_vel_des_, SJ_VEC, "body_ang_vel_des", 3);
  data_manager->RegisterData(&pelvis_ang_vel_, SJ_VEC, "pelvis_ang_vel", 3);
  data_manager->RegisterData(&body_ang_vel_, SJ_VEC, "body_ang_vel", 3);

  // RIGHT FOOT
  // [right Foot] linear
  data_manager->RegisterData(&rfoot_pos_des_, VECT3, "rfoot_pos_des", 3);
  data_manager->RegisterData(&rfoot_vel_des_, VECT3, "rfoot_vel_des", 3);
  data_manager->RegisterData(&rfoot_acc_des_, VECT3, "rfoot_acc_des", 3);
  data_manager->RegisterData(&rfoot_pos_, VECT3, "rfoot_pos", 3);
  data_manager->RegisterData(&rfoot_vel_, VECT3, "rfoot_vel", 3);
  // [right Foot] angular
  data_manager->RegisterData(&rfoot_ori_des_, QUATERNION, "rfoot_ori_des", 4);
  data_manager->RegisterData(&rfoot_ang_vel_des_, VECT3, "rfoot_ang_vel_des", 3);
  data_manager->RegisterData(&rfoot_ori_, QUATERNION, "rfoot_ori", 4);
  data_manager->RegisterData(&rfoot_ang_vel_, VECT3, "rfoot_ang_vel", 3);

  // LEFT FOOT
  // [left Foot] linear
  data_manager->RegisterData(&lfoot_pos_des_, VECT3, "lfoot_pos_des", 3);
  data_manager->RegisterData(&lfoot_vel_des_, VECT3, "lfoot_vel_des", 3);
  data_manager->RegisterData(&lfoot_acc_des_, VECT3, "lfoot_acc_des", 3);
  data_manager->RegisterData(&lfoot_pos_, VECT3, "lfoot_pos", 3);
  data_manager->RegisterData(&lfoot_vel_, VECT3, "lfoot_vel", 3);
  // [left Foot] angular
  data_manager->RegisterData(&lfoot_ori_des_, QUATERNION, "lfoot_ori_des", 4);
  data_manager->RegisterData(&lfoot_ang_vel_des_, VECT3, "lfoot_ang_vel_des", 3);
  data_manager->RegisterData(&lfoot_ori_, QUATERNION, "lfoot_ori", 4);
  data_manager->RegisterData(&lfoot_ang_vel_, VECT3, "lfoot_ang_vel", 3);

  data_manager->RegisterData(&Fr_, SJ_VEC, "reaction_force", 24);
  data_manager->RegisterData(&cent_ang_vel_, SJ_VEC, "cent_ang_vel", 3);
}

void StateProvider::SaveDoubleReactionForce(const sejong::Vector & Fr){
  Fr_ = Fr;
}

void StateProvider::SaveRightSingleReactionForce(const sejong::Vector & Fr){
  sejong::Vector zero(12);
  sejong::Vector save(24);
  zero.setZero();
  save.setZero();
  save.head(12) = zero;
  save.tail(12) = Fr;
  Fr_ = save;
}

void StateProvider::SaveLeftSingleReactionForce(const sejong::Vector & Fr){
  sejong::Vector zero(12);
  sejong::Vector save(24);
  zero.setZero();
  save.setZero();
  save.head(12) = Fr;
  save.tail(12) = zero;
  Fr_ = save;
}

void StateProvider::Push_PathNode(const sejong::Vector & node){
  path_.push_back(node);
}

void StateProvider::SaveStatEqConvexHull(const sejong::Vect3 & point){
    stat_eq_convex_hull.push_back(point);
}

void StateProvider::Save3DCoMAccConvexHull(const sejong::Vect3 & point) {
    comacc_convex_hull.push_back(point);
}

void StateProvider::SaveCoMPos(const sejong::Vect3 & point) {
   CoM_pos_ = point;
}

void StateProvider::SaveCoMAcc(const sejong::Vector & acc) {
    CoM_acc = acc;
}

void StateProvider::CleaningList(){
  path_.clear();
  foot_placement_list_.clear();
  foot_orientation_list_.clear();
  stat_eq_convex_hull.clear();
  comacc_convex_hull.clear();
}

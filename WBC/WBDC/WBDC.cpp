#include "WBDC.hpp"
#include <Utils/utilities.hpp>

WBDC::WBDC(const std::vector<bool> & act_list): WBC(act_list){}

void WBDC::UpdateSetting(const sejong::Matrix & A,
                         const sejong::Matrix & Ainv,
                         const sejong::Vector & cori,
                         const sejong::Vector & grav,
                         void* extra_setting){
  A_ = A;
  Ainv_ = Ainv;
  cori_ = cori;
  grav_ = grav;
}

void WBDC::MakeTorque(const std::vector<Task*> & task_list,
                      const std::vector<ContactSpec*> & contact_list,
                      sejong::Vector & cmd,
                      void* extra_input){
  _PrintDebug(0);
  data_ = static_cast<WBDC_ExtraData*>(extra_input);

  // Contact & Task Setting
  _ContactBuilding(contact_list);
  _PrintDebug(1);

  _TaskHierarchyBuilding(task_list);
  _PrintDebug(2);

  // Dimension Setting
  dim_cam_ = (data_->Icam).rows();
  dim_opt_ = dim_rf_ + dim_relaxed_task_;
  // dim_eq_cstr_ = num_passive_ + dim_cam_;
  dim_eq_cstr_ = num_passive_;
  dim_ieq_cstr_ = dim_rf_cstr_ + 2*num_act_joint_;

  // Matrix Setting
  _MatrixInitialization();
  _PrintDebug(3);

  // Equality Constraint Setting
  _SetEqualityConstraint();
  _PrintDebug(4);

  // Inequality Constraint Setting
  _SetInEqualityConstraint();
  _PrintDebug(5);

  // printf("G:\n");
  // std::cout<<G<<std::endl;
  // printf("g0:\n");
  // std::cout<<g0<<std::endl;

  // printf("CE:\n");
  // std::cout<<CE<<std::endl;
  // printf("ce0:\n");
  // std::cout<<ce0<<std::endl;

  // printf("CI:\n");
  // std::cout<<CI<<std::endl;
  // printf("ci0:\n");
  // std::cout<<ci0<<std::endl;

  std::cout << "f: " << solve_quadprog(G, g0, CE, ce0, CI, ci0, z) << std::endl;
  std::cout << "x: " << z << std::endl;

  _GetSolution(cmd);
}

void WBDC::_SetEqualityConstraint(){
  sejong::Matrix sj_CE(dim_eq_cstr_, dim_opt_);
  sejong::Vector sj_ce0(dim_eq_cstr_);
  sj_CE.setZero();
  sj_ce0.setZero();
  // Virtual Torque
  sj_CE.block(0,0, num_passive_, dim_opt_) = Sv_ * tot_tau_Mtx_;
  sj_ce0.head(num_passive_) = Sv_ * tot_tau_Vect_;

  // Centroidal Angular Momentum
  // if(dim_relaxed_task_ > 0){
  //   sj_CE.block(num_passive_, 0, dim_cam_, dim_rf_) = - Wf_;
  //   sj_CE.block(num_passive_, dim_rf_, dim_cam_, dim_cam_) = data_->Icam * data_->Jcam * B_ * S_delta_;

  // }else {
  //   sj_CE.block(num_passive_, 0, dim_cam_, dim_opt_) = -Wf_;
  // }
  // sj_ce0.tail(dim_cam_) = data_->Icam * data_->Jcam * B_ * task_cmd_ + data_->Icam * data_->JcamDotQdot;


  for(int i(0); i< dim_eq_cstr_; ++i){
    for(int j(0); j<dim_opt_; ++j){
      CE[j][i] = sj_CE(i,j);
    }
    ce0[i] = sj_ce0[i];
  }

  sejong::pretty_print(sj_CE, std::cout, "WBDC: CE");
  sejong::pretty_print(sj_ce0, std::cout, "WBDC: ce0");
}

void WBDC::_SetInEqualityConstraint(){
  sejong::Matrix sj_CI(dim_ieq_cstr_, dim_opt_);
  sejong::Vector sj_ci0(dim_ieq_cstr_);
  sj_CI.setZero();

  // RF constraint
  sj_CI.block(0,0, dim_rf_cstr_, dim_rf_) = Uf_;
  (sj_ci0.head(dim_rf_cstr_)).setZero();

  // Torque min & max
  // min
  sj_CI.block(dim_rf_cstr_, 0, num_act_joint_, dim_opt_) = Sa_ * tot_tau_Mtx_;
  sj_ci0.segment(dim_rf_cstr_, num_act_joint_) = Sa_ * tot_tau_Vect_ - data_->tau_min;

  // max
  sj_CI.block(dim_rf_cstr_ + num_act_joint_, 0, num_act_joint_, dim_opt_) = -Sa_ * tot_tau_Mtx_;
  sj_ci0.tail(num_act_joint_) = -Sa_ * tot_tau_Vect_ + data_->tau_max;

  for(int i(0); i< dim_ieq_cstr_; ++i){
    for(int j(0); j<dim_opt_; ++j){
      CI[j][i] = sj_CI(i,j);
    }
    ci0[i] = sj_ci0[i];
  }
  sejong::pretty_print(sj_CI, std::cout, "WBDC: CI");
  sejong::pretty_print(sj_ci0, std::cout, "WBDC: ci0");
}

void WBDC::_ContactBuilding(const std::vector<ContactSpec*> & contact_list){
  sejong::Matrix Uf, Wf;
  // Initial
  sejong::Matrix Jc;
  sejong::Vector JcDotQdot;
  contact_list[0]->getContactJacobian(Jc);
  contact_list[0]->getJcDotQdot(JcDotQdot);
  Jc_ = Jc;
  JcDotQdot_ = JcDotQdot;
  static_cast<WBDC_ContactSpec*>(contact_list[0])->getRFConstraintMtx(Uf_);
  static_cast<WBDC_ContactSpec*>(contact_list[0])->getCMProjectionMtx(Wf_);
  dim_rf_ = contact_list[0]->getDim();
  dim_rf_cstr_ = static_cast<WBDC_ContactSpec*>(contact_list[0])->getDimRFConstratint();

  int dim_new_rf, dim_new_rf_cstr;

  for(int i(1); i<contact_list.size(); ++i){
    contact_list[i]->getContactJacobian(Jc);
    contact_list[i]->getJcDotQdot(JcDotQdot);
    dim_new_rf = contact_list[i]->getDim();
    dim_new_rf_cstr = static_cast<WBDC_ContactSpec*>(contact_list[i])->getDimRFConstratint();

    // Jc append
    Jc_.conservativeResize(dim_rf_ + dim_new_rf, num_qdot_);
    Jc_ = Jc_.block(dim_rf_, 0, dim_new_rf, num_qdot_) = Jc;

    // JcDotQdot append
    JcDotQdot_.conservativeResize(dim_rf_ + dim_new_rf, 1);
    JcDotQdot_.tail(dim_new_rf) = JcDotQdot;


    // Uf
    static_cast<WBDC_ContactSpec*>(contact_list[i])->getRFConstraintMtx(Uf);
    Uf_.conservativeResize(dim_rf_cstr_ + dim_new_rf_cstr, dim_rf_ + dim_new_rf);
    Uf_.block(0, dim_rf_, dim_rf_cstr_, dim_new_rf).setZero();
    Uf_.block(dim_rf_cstr_, 0, dim_new_rf_cstr, dim_rf_).setZero();
    Uf_.block(dim_rf_cstr_, dim_rf_, dim_new_rf_cstr, dim_new_rf) = Uf;

    // Wf
    static_cast<WBDC_ContactSpec*>(contact_list[i])->getCMProjectionMtx(Wf);
    Wf_.conservativeResize(dim_cam_, dim_rf_ + dim_new_rf);
    Wf_.block(0, dim_rf_, dim_cam_, dim_new_rf) = Wf;

    dim_rf_ += dim_new_rf;
    dim_rf_cstr_ += dim_new_rf_cstr;
  }

  sejong::pretty_print(Jc_, std::cout, "WBDC: Jc");
  sejong::pretty_print(JcDotQdot_, std::cout, "WBDC: JcDot Qdot");
  sejong::pretty_print(Uf_, std::cout, "WBDC: Uf");
  sejong::pretty_print(Wf_, std::cout, "WBDC: Wf");
}

void WBDC::_TaskHierarchyBuilding(const std::vector<Task*> & task_list){
  dim_relaxed_task_ = 0;

  sejong::Matrix Jt, JtPre;
  sejong::Matrix Jt_inv, JtPre_inv;
  sejong::Vector JtDotQdot;
  sejong::Vector xddot;
  sejong::Matrix Npre;
  sejong::Matrix I_JtPreInv_Jt;
  Task* task = task_list[0];

  int tot_task_size(0);

  // First Task: Contact Constraint
  Jt = Jc_;
  JtDotQdot = JcDotQdot_;

  _WeightedInverse(Jt, Ainv_, Jt_inv);
  B_ = Jt_inv;
  c_ = Jt_inv * JtDotQdot;
  task_cmd_ = sejong::Vector::Zero(dim_rf_);
  Npre = sejong::Matrix::Identity(num_qdot_, num_qdot_) - Jt_inv * Jt;
  tot_task_size += dim_rf_;

  sejong::pretty_print(B_, std::cout, "WBDC: B");
  sejong::pretty_print(c_, std::cout, "WBDC: c");

  // Task Stacking
  for(int i(0); i<task_list.size(); ++i){
    // Obtaining Task
    task = task_list[i];
    task->getTaskJacobian(Jt);
    task->getTaskJacobianDotQdot(JtDotQdot);
    task->getCommand(xddot);
    JtPre = Jt * Npre;
    _WeightedInverse(JtPre, Ainv_, JtPre_inv);
    I_JtPreInv_Jt = sejong::Matrix::Identity(num_qdot_, num_qdot_) - JtPre_inv * Jt;

    // B matrix building
    printf("task size (total, current): %i, %i\n", tot_task_size, task->getDim());
    B_.conservativeResize(num_qdot_, tot_task_size + task->getDim());
    B_.block(0, 0, num_qdot_, tot_task_size) =
      I_JtPreInv_Jt * B_.block(0, 0, num_qdot_, tot_task_size);
    B_.block(0, tot_task_size, num_qdot_, task->getDim()) = JtPre_inv;

    // c vector building
    c_ = I_JtPreInv_Jt * c_ - JtPre_inv * JtDotQdot;

    // task commad
    task_cmd_.conservativeResize(tot_task_size + task->getDim());
    task_cmd_.tail(task->getDim()) = xddot;

    // Build for Next
    Npre = Npre * ( sejong::Matrix::Identity(num_qdot_, num_qdot_) - JtPre_inv * JtPre);
    tot_task_size += task->getDim();

    sejong::pretty_print(B_, std::cout, "WBDC: B");
    sejong::pretty_print(c_, std::cout, "WBDC: c");
    sejong::pretty_print(task_cmd_, std::cout, "WBDC: task cmd");
    // Relaxed Task Setup
    if(static_cast<WBDC_Task*>(task)->getRelaxed_Dim() > 0){
      sejong::Matrix Sd;
      static_cast<WBDC_Task*>(task)->getSdelta(Sd);
      int dim_relax = static_cast<WBDC_Task*>(task)->getRelaxed_Dim();

      if(dim_relaxed_task_ == 0){ // First Visit
        S_delta_ = sejong::Matrix::Zero(tot_task_size, dim_relax);
        S_delta_.block(tot_task_size - task->getDim(), 0, task->getDim(), dim_relax) = Sd;
      } else { // Next Visit
        S_delta_.conservativeResize(tot_task_size, dim_relaxed_task_ + dim_relax);
        (S_delta_.block(tot_task_size - task->getDim(), 0, task->getDim(), dim_relaxed_task_)).setZero();
        S_delta_.block(tot_task_size - task->getDim(), dim_relaxed_task_, task->getDim(), dim_relax) = Sd;
      }
      dim_relaxed_task_ += dim_relax;

      sejong::pretty_print(S_delta_, std::cout, "tot S delta");
    }
  }

}

void WBDC::_GetSolution(sejong::Vector & cmd){
  sejong::Vector result(dim_opt_);
  for(int i(0); i<dim_opt_; ++i) result[i] = z[i];
  sejong::Vector tot_tau = tot_tau_Mtx_*result + tot_tau_Vect_;
  cmd = tot_tau.tail(num_act_joint_);

  sejong::pretty_print(result, std::cout, "opt result");
  sejong::pretty_print(tot_tau, std::cout, "tot tau result");
}

void WBDC::_MatrixInitialization(){

  if(dim_relaxed_task_ > 0){
    tot_tau_Mtx_.resize(num_qdot_, dim_opt_);
    tot_tau_Mtx_.block(0,0, num_qdot_, dim_rf_) = -Jc_.transpose();
    tot_tau_Mtx_.block(0, dim_rf_, num_qdot_, dim_relaxed_task_) = A_ * B_ * S_delta_;
  } else {
    tot_tau_Mtx_ =  -Jc_.transpose();
  }
  tot_tau_Vect_ = A_*B_*task_cmd_ + A_*c_ + cori_ + grav_;

  sejong::pretty_print(tot_tau_Mtx_, std::cout, "tot tau matrix");
  sejong::pretty_print(tot_tau_Vect_, std::cout, "tot tau Vector");

  G.resize(dim_opt_, dim_opt_);
  g0.resize(dim_opt_);
  CE.resize(dim_opt_, dim_eq_cstr_);
  ce0.resize(dim_eq_cstr_);
  CI.resize(dim_opt_, dim_ieq_cstr_);
  ci0.resize(dim_ieq_cstr_);

  for(int i(0); i<dim_opt_; ++i){
    for(int j(0); j<dim_opt_; ++j){
      G[i][j] = 0.;
    }
    g0[i] = 0.;
  }

  // Set Cost
  for (int i(0); i < dim_opt_; ++i){
    G[i][i] = data_->cost_weight[i];
  }
}

#ifndef WHOLE_BODY_DYNAMIC_CONTROL_H
#define WHOLE_BODY_DYNAMIC_CONTROL_H

#include <WBC.hpp>
#include <Utils/utilities.hpp>
#include <Optimizer/Goldfarb/QuadProg++.hh>

#include "WBDC_Task.hpp"
#include "WBDC_ContactSpec.hpp"

class WBDC_ExtraData{
 public:
  sejong::Matrix Icam;
  sejong::Matrix Jcam;
  sejong::Matrix JcamDotQdot;
  sejong::Vector cost_weight;

  WBDC_ExtraData(){}
  ~WBDC_ExtraData(){}
};

class WBDC: public WBC{
 public:
 WBDC(int num_act_joint, int num_qdot, const std::vector<bool> & act_list): WBC(num_act_joint, num_qdot, act_list), num_act_joint_(num_act_joint)
  {
    num_act_joint_ = num_act_joint;
    num_virtual_ = num_qdot - num_act_joint;
  }
  virtual ~WBDC(){}

  virtual void MakeTorque(std::vector<Task*> task_list,
                          std::vector<ContactSpec*> contact_list,
                          sejong::Vector & cmd,
                          void* extra_input = NULL){
    // Test
    cmd.setZero();
    WBDC_ExtraData* data = static_cast<WBDC_ExtraData*>(extra_input);

    // Dimension Setting
    dim_rf_ = 0;
    dim_rf_cstr_ = 0;
    for(int i(0); i<contact_list.size(); ++i){
      dim_rf_ += contact_list[i]->getDim();
      dim_rf_cstr_ += static_cast<WBDC_ContactSpec*>(contact_list[i])->getDimRFConstratint();
    }

    dim_relaxed_task_ = 0;
    for(int i(0); i<dim_relaxed_task_; ++i){
      dim_relaxed_task_ += static_cast<WBDC_Task*>(task_list[i])->getRelaxed_Dim();
    }

    dim_cam_ = (data->Icam).rows();

    G.resize(dim_rf_ + dim_relaxed_task_, dim_rf_ + dim_relaxed_task_);
    g0.resize(dim_rf_ + dim_relaxed_task_);
    CE.resize(dim_rf_ + dim_relaxed_task_, dim_cam_ + num_virtual_);
    ce0.resize(dim_cam_ +  num_virtual_);
    CI.resize(dim_rf_ + dim_relaxed_task_, dim_rf_cstr_ + 2*num_act_joint_);
    ci0.resize(dim_rf_cstr_ + 2*num_act_joint_);

    // Set Cost
    for (int i(0); i < dim_relaxed_task_ + dim_rf_; ++i){
      G[i][i] = data->cost_weight[i];
    }
  }

private:
  GolDIdnani::GVect<double> x;
  // Cost
  GolDIdnani::GMatr<double> G;
  GolDIdnani::GVect<double> g0;

  // Equality
  GolDIdnani::GMatr<double> CE;
  GolDIdnani::GVect<double> ce0;

  // Inequality
  GolDIdnani::GMatr<double> CI;
  GolDIdnani::GVect<double> ci0;

  int num_virtual_;
  int num_act_joint_;
  int dim_rf_;
  int dim_relaxed_task_;
  int dim_cam_;
  int dim_rf_cstr_;

  void _PrintDebug(double i) {
    printf("[WBDC] %f \n", i);
  }

};

#endif

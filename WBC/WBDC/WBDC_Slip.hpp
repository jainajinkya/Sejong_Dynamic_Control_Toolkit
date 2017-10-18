#ifndef WHOLE_BODY_DYNAMIC_CONTROL_SLIP_H
#define WHOLE_BODY_DYNAMIC_CONTROL_SLIP_H

#include <WBC.hpp>
#include <Utils/utilities.hpp>
#include <Optimizer/Goldfarb/QuadProg++.hh>

#include "WBDC_Task.hpp"
#include "WBDC_Slip_ContactSpec.hpp"
#include "WBDC.hpp"

class WBDC_Slip: public WBC{
 public:
  WBDC_Slip(const std::vector<bool> & act_list);
  virtual ~WBDC_Slip(){}

  virtual void UpdateSetting(const sejong::Matrix & A,
                             const sejong::Vector & cori,
                             const sejong::Vector & grav,
                             void* extra_setting = NULL);

  virtual void MakeTorque(std::vector<Task*> task_list,
                          std::vector<ContactSpec*> contact_list,
                          sejong::Vector & cmd,
                          void* extra_input = NULL);

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
    printf("[WBDC Slip] %f \n", i);
  }

};

#endif

#include "WBDC_Slip.hpp"
#include <Optimizer/gurobi/src/gurobi_c++.h>

WBDC_Slip::WBDC_Slip(const std::vector<bool> & act_list): WBC(act_list){}

void WBDC_Slip::UpdateSetting(const sejong::Matrix & A,
                         const sejong::Vector & cori,
                         const sejong::Vector & grav,
                         void* extra_setting){
  A_ = A;
  cori_ = cori;
  grav_ = grav;
}

void WBDC_Slip::MakeTorque(std::vector<Task*> task_list,
                           std::vector<ContactSpec*> contact_list,
                           sejong::Vector & cmd,
                           void* extra_input){
    // Test
    cmd.setZero();
    WBDC_ExtraData* data = static_cast<WBDC_ExtraData*>(extra_input);

    // Dimension Setting
    dim_rf_ = 0;
    dim_rf_cstr_ = 0;
    for(int i(0); i<contact_list.size(); ++i){
      dim_rf_ += contact_list[i]->getDim();
      dim_rf_cstr_ += static_cast<WBDC_Slip_ContactSpec*>(contact_list[i])->getDimRFConstratint();
    }

    dim_relaxed_task_ = 0;
    for(int i(0); i<dim_relaxed_task_; ++i){
      dim_relaxed_task_ += static_cast<WBDC_Task*>(task_list[i])->getRelaxed_Dim();
    }

    dim_cam_ = (data->Icam).rows();
}

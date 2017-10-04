#include "LIPM_Plot_System.h"
#include <environment/LIPM_3D.h>
#include <Utils/utilities.h>
#include <agent/LIPM_Tester.h>
#include <stdio.h>

LIPM_Plot_System::LIPM_Plot_System(LIPM_3D* lipm, const LIPM_Tester* tester):
  lipm_(lipm),
  num_data_send_(0),
  step_idx_(0)
{
  tester->getTestInformation(ini_time_list_, ini_state_list_, pivot_list_);
  end_time_ = *ini_time_list_.rbegin();
  printf("[LIPM Plotting System] Contructed\n");
}
LIPM_Plot_System::~LIPM_Plot_System(){
}


void LIPM_Plot_System::UpdateData(double global_time, std::vector<double> & data){
  sejong::Vector global_ini_state;
  sejong::Vector local_ini_state;
  sejong::Vector local_state;

  sejong::Vector pivot;
  double local_time;

  _find_ini_state_curr_time(global_time, global_ini_state, pivot, local_time);
  data.resize(NUM_VALUE_PYTHON);

  data[0] = num_data_send_;
  data[1] = global_time;

  local_ini_state = global_ini_state;
  local_state = global_ini_state;

  sejong::Vector tmp_local_ini_state(4);
  sejong::Vector tmp_local_curr_state(4);
  for (int i(0); i<2; ++i){
    tmp_local_ini_state[i] = local_ini_state[i] - pivot[i];
    tmp_local_ini_state[i + 2] = local_ini_state[3+i];
  }

  lipm_->getState(tmp_local_ini_state, local_time, tmp_local_curr_state);

  for (int i(0); i<2; ++i){
    local_state[i] = tmp_local_curr_state[i];
    local_state[i + 3] = tmp_local_curr_state[2+i];
  }


  for(int i(0); i<3; ++i){
    data[2 + i] = local_state[i] + pivot[i];
    data[5 + i] = local_state[3 + i];
  }
  data[8] = pivot[0];
  data[9] = pivot[1];
  data[10] = pivot[2];

  // if(num_data_send_ % 30 == 0){
  //   sejong::pretty_print(data,"data");
  // }
  if (global_time > (*ini_time_list_.rbegin()) - 0.01){
      sejong::pretty_print(data,"data");
  }
  ++num_data_send_;
}

void LIPM_Plot_System::_find_ini_state_curr_time(double global_time,
                                                 sejong::Vector & state,
                                                 sejong::Vector & pivot,
                                                 double & local_time){
  local_time = global_time - ini_time_list_[step_idx_];
  state = ini_state_list_[step_idx_];
  pivot = pivot_list_[step_idx_];

  if (global_time > *ini_time_list_.rbegin()){
    num_data_send_ = -1;
    step_idx_ = 0;
  } else if (global_time > ini_time_list_[step_idx_+1]){
    ++step_idx_;
  }
}

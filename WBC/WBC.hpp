#ifndef WHOLE_BODY_CONTROLLER
#define WHOLE_BODY_CONTROLLER

#include <Utils/wrap_eigen.hpp>
#include "Task.hpp"
#include "ContactSpec.hpp"

class WBC{
public:
  WBC(int num_act_joint, int num_qdot, const std::vector<bool> & act_list):
    Sa_(num_qdot, num_act_joint),
    Sv_(num_qdot, num_act_joint)
  {
    // Set virtual & actuated selection matrix
    int j(0);
    for(int i(0); i <num_qdot; ++i){
      if(act_list[i] == true){
        Sa_(i, j) = 1.;
        ++j;
      }
      else Sv_(i,j) = 1.;
    }
  }
  virtual ~WBC(){}

  virtual void UpdateSetting(const sejong::Matrix & A,
                             const sejong::Vector & cori,
                             const sejong::Vector & grav,
                             void* extra_setting = NULL) = 0;
  virtual void MakeTorque(const std::vector<Task*> task_list,
                          const std::vector<ContactSpec*> contact_list,
                          void* extra_input = NULL) =0;

protected:
  sejong::Matrix Sa_; // Actuated joint
  sejong::Matrix Sv_; // Virtual joint

  sejong::Matrix A_;
  sejong::Vector cori_;
  sejong::Vector grav_;
};

#endif

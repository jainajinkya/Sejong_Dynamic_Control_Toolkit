#ifndef WHOLE_BODY_CONTROLLER
#define WHOLE_BODY_CONTROLLER

#include <Utils/wrap_eigen.hpp>
#include <Utils/utilities.hpp>

#include "Task.hpp"
#include "ContactSpec.hpp"

class WBC{
public:
  WBC(const std::vector<bool> & act_list):num_act_joint_(0),
                                          num_passive_(0)
  {
    num_qdot_ = act_list.size();
    for(int i(0); i<num_qdot_; ++i){
      if(act_list[i] == true) ++num_act_joint_;
      else ++num_passive_;
    }
    Sa_ = sejong::Matrix::Zero(num_act_joint_, num_qdot_);
    Sv_ = sejong::Matrix::Zero(num_passive_, num_qdot_);

    // Set virtual & actuated selection matrix
    int j(0);
    int k(0);
    for(int i(0); i <num_qdot_; ++i){
      if(act_list[i] == true){
        Sa_(j, i) = 1.;
        ++j;
      }
      else{
        Sv_(k,i) = 1.;
        ++k;
      }
    }
    // sejong::pretty_print(Sa_, std::cout, "Sa");
    // sejong::pretty_print(Sv_, std::cout, "Sv");
  }
  virtual ~WBC(){}

  virtual void UpdateSetting(const sejong::Matrix & A,
                             const sejong::Matrix & Ainv,
                             const sejong::Vector & cori,
                             const sejong::Vector & grav,
                             void* extra_setting = NULL) = 0;

  virtual void MakeTorque(const std::vector<Task*> & task_list,
                          const std::vector<ContactSpec*> & contact_list,
                          sejong::Vector & cmd,
                          void* extra_input = NULL) =0;

protected:
  // full rank fat matrix only
  void _WeightedInverse(const sejong::Matrix & J,
                        const sejong::Matrix & Winv,
                        sejong::Matrix & Jinv){
    sejong::Matrix lambda(J* Winv * J.transpose());
    Jinv = Winv * J.transpose() * lambda.inverse();
  }

  int num_qdot_;
  int num_act_joint_;
  int num_passive_;

  sejong::Matrix Sa_; // Actuated joint
  sejong::Matrix Sv_; // Virtual joint

  sejong::Matrix A_;
  sejong::Matrix Ainv_;
  sejong::Vector cori_;
  sejong::Vector grav_;
};

#endif

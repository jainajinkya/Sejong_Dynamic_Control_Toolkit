#ifndef LIPM_TESTER
#define LIPM_TESTER


#include <Utils/wrap_eigen.hpp>
#include <vector>

class Agent;
class LIPM_3D;

class LIPM_Tester{
 public:
  LIPM_Tester(){
    ini_time_list_.clear();
    ini_state_list_.clear();
    pivot_list_.clear();
  }
  virtual ~LIPM_Tester(){}

  void getTestInformation(std::vector<double> & ini_time_list,
                          std::vector<sejong::Vector> & ini_state_list,
                          std::vector<sejong::Vector> & pivot_list) const{
    ini_time_list = ini_time_list_;
    ini_state_list = ini_state_list_;
    pivot_list = pivot_list_;
  }
  virtual void Build_Test_Information(LIPM_3D* lipm, Agent* agent) = 0;

 protected:
  std::vector<double> ini_time_list_;
  std::vector<sejong::Vector> ini_state_list_;
  std::vector<sejong::Vector> pivot_list_;
};

#endif

#ifndef MERCURY_CONTROLLER
#define MERCURY_CONTROLLER

#include <Utils/wrap_eigen.hpp>
#include <string>

class MercuryModel;
class StateProvider;
class Task;
class ContactSpec;

class MercuryController{
public:
  MercuryController();
  virtual ~MercuryController();

  virtual void OneStep(sejong::Vector & gamma) = 0;
  virtual void FirstVisit() = 0;
  virtual void LastVisit() = 0;
  virtual bool EndOfPhase() = 0;

  virtual void CtrlInitialization(std::string setting_file_name) = 0;

protected:
  void _PreProcessing_Command();
  void _PostProcessing_Command(sejong::Vector & gamma);
  void _DynConsistent_Inverse(const sejong::Matrix & J, sejong::Matrix & Jinv);

  StateProvider* sp_;
  MercuryModel* robot_model_;

  sejong::Matrix A_;
  sejong::Matrix Ainv_;
  sejong::Vector grav_;
  sejong::Vector coriolis_;

  std::vector<Task*> task_list_;
  std::vector<ContactSpec*> contact_list_;
  std::vector<bool> act_list_;

  double state_machine_time_;
  double ctrl_start_time_;
};

#endif

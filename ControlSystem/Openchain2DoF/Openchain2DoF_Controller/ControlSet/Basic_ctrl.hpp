#ifndef OPENCHAIN_2DOF_BASIC_CONTROLLER_H
#define OPENCHAIN_2DOF_BASIC_CONTROLLER_H

#include <Openchain2DoF_Controller/OC2Controller.hpp>

class Basic_ctrl: public OC2Controller{
public:
  Basic_ctrl();
  virtual ~Basic_ctrl();

  virtual void ComputeTorqueCommand(sejong::Vector & gamma);
  virtual void Initialization();

protected:
  // Functions
  void _jpos_ctrl(sejong::Vector & gamma);
  void _ee_ctrl(sejong::Vector & gamma);

  sejong::Vect3 ee_ini_;

  // Data Save
  sejong::Vector des_pos_;
  sejong::Vector act_pos_;
  sejong::Vector act_vel_;
  sejong::Vector xddot_cmd_;
  
  int count_command_;
  double state_machine_time_;
  double start_time_;

  sejong::Vector jpos_ini_;
  sejong::Vector jpos_des_;

  SJLinkID link_id_;
  int initialization_count_;

};


#endif

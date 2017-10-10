#ifndef OPENCHAIN_2DOF_DDP_CONTROLLER_H
#define OPENCHAIN_2DOF_DDP_CONTROLLER_H

#include <Openchain2DoF_Controller/OC2Controller.hpp>
#include <Simulator/RBDL_simulator/Systems/Openchain2DoF/OC2_Sim_Model.hpp>

class DDP_ctrl: public OC2Controller{
public:
  DDP_ctrl();
  virtual ~DDP_ctrl();

  virtual void ComputeTorqueCommand(sejong::Vector & gamma);
  virtual void Initialization();

protected:
  // Functions
  void _zero_ctrl(sejong::Vector & gamma); 
  void _jpos_ctrl(sejong::Vector & gamma);
  void _ee_ctrl(sejong::Vector & gamma);

  void _internal_simulate(const sejong::Vector & x_state, const sejong::Vector & u_in, sejong::Vector & x_next_state); // x_{t+1} = f(x, gamma(u))
  void _l_running_cost(const sejong::Vector & x_state, const sejong::Vector & u_in, double & cost);
  void _l_final_cost(const sejong::Vector & x_state_final, double & cost);  
  void _gradient_finite_difference();
  void _hessian_finite_difference();  
  void _mpc_ctrl(const sejong::Vector & gamma);

  sejong::Vect3 ee_ini_;

  // Get Internal Model of the robot for MPC
  OC2_Sim_Model* internal_model;
  std::vector<sejong::Vector> x_sequence;  
  std::vector<sejong::Vector> u_sequence;
  sejong::Vector q_temp;
  sejong::Vector qdot_temp;  

  int N_horizon; // Control Horizon
  double mpc_time_step;


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

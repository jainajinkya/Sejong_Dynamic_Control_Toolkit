#ifndef WALKER_2D_DDP_CONTROLLER_H
#define WALKER_2D_DDP_CONTROLLER_H

#include <Walker2D_Controller/Walker2D_Controller.hpp>
#include <Walker2D_Model/Walker2D_Model.hpp>
#include <Walker2D_Model/Walker2D_Dyn_Model.hpp>
#include <Walker2D_Model/Walker2D_Kin_Model.hpp>

#include <Optimizer/Goldfarb/QuadProg++.hh>
#include "iLQR.hpp"

#define STATE_X_SIZE NUM_Q + NUM_QDOT

class DDP_ctrl: public Walker2D_Controller{
public:
  DDP_ctrl();
  virtual ~DDP_ctrl();

  virtual void ComputeTorqueCommand(sejong::Vector & gamma);
  virtual void Initialization();

  // Computes x_{t+1} = f(x_t, u_t)
  sejong::Vector f(const sejong::Vector & x, const sejong::Vector & u); 
  // Computes l(x, u)   
  double l_cost(const sejong::Vector &x, const sejong::Vector &u);  
  // Computes l_F(x)  
  double l_cost_final(const sejong::Vector &x);

  double custom_J_cost(const std::vector<sejong::Vector> & X, const std::vector<sejong::Vector> & U);

protected:
  const double NEAR_ZERO = std::sqrt(std::numeric_limits<double>::epsilon());
  // Functions
  void _DDP_ctrl(sejong::Vector & gamma);  
  void _jpos_ctrl(sejong::Vector & gamma);
  sejong::Vect3 ee_ini_;

  void _update_internal_model(const sejong::Vector & x_state);
  void _get_WBC_command(const sejong::Vector & x_state, 
                        const sejong::Vector & des_acc, 
                        sejong::Vector & gamma_int); 

  iLQR* ilqr_;
  // Get Internal Model of the robot for DDP
  Walker2D_Model* internal_model;

  std::vector<sejong::Vector> gamma_sequence;  
  sejong::Matrix Q;
  sejong::Matrix N;
  sejong::Matrix T;    

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

#ifndef WALKER_2D_DDP_CONTROLLER_H
#define WALKER_2D_DDP_CONTROLLER_H

#include <Walker2D_Controller/Walker2D_Controller.hpp>
#include "iLQR.hpp"

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

  iLQR* ilqr_;

protected:
  // Functions
  void _DDP_ctrl(sejong::Vector & gamma);  
  void _jpos_ctrl(sejong::Vector & gamma);
  sejong::Vect3 ee_ini_;

  void   get_WBC_command(const sejong::Vector & x_state, 
                         const sejong::Vector & des_acc, 
                         sejong::Vector & gamma_int); 

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

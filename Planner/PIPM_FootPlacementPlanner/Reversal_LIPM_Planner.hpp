#ifndef VELOCITY_REVERSAL_PLANNER_LINEAR_INVERTED_PENDULUM
#define VELOCITY_REVERSAL_PLANNER_LINEAR_INVERTED_PENDULUM

#include <Planner/Planner.hpp>
#include <vector>

class ParamReversalPL{
public:
  double swing_time;
};

class Reversal_LIPM_Planner: public Planner{
public:
  Reversal_LIPM_Planner();
  virtual ~Reversal_LIPM_Planner();

  virtual void PlannerInitialization(const std::string & setting_file);
  
  virtual void getNextFootLocation(const sejong::Vect3 & com_pos,
                                   const sejong::Vect3 & com_vel,
                                   sejong::Vect3 & target_pos,
                                   const void* additional_input = NULL,
                                   void* additional_output = NULL);

  // Set Functions
  void setOmega(double com_height){
    b_set_omega_ = true;
    omega_ = 9.81/com_height;
  }

protected:
  double t_prime_x_;
  double t_prime_y_;
  double kappa_x_;
  double kappa_y_;
  double step_length_limit_;
  

  std::vector<double> com_vel_limit_;

  double omega_;
  bool b_set_omega_;

  void _computeSwitchingState(double swing_time,
                              const sejong::Vect3& com_pos,
                              const sejong::Vect3& com_vel,
                              sejong::Vect2 & switching_x_state,
                              sejong::Vect2 & switching_y_state);
};

#endif

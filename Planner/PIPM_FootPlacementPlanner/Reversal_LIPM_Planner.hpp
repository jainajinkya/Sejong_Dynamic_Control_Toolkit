#ifndef VELOCITY_REVERSAL_PLANNER_LINEAR_INVERTED_PENDULUM
#define VELOCITY_REVERSAL_PLANNER_LINEAR_INVERTED_PENDULUM

#include <Planner/Planner.hpp>
#include <vector>

class ParamReversalPL{
public:
  double swing_time;
  sejong::Vect2 des_loc;
  sejong::Vect3 stance_foot_loc;
  bool b_positive_sidestep;
};

class OutputReversalPL{
public:
  double time_modification;
};

class Reversal_LIPM_Planner: public Planner{
public:
  Reversal_LIPM_Planner();
  virtual ~Reversal_LIPM_Planner();

  virtual void PlannerInitialization(const std::string & setting_file);
  
  virtual void getNextFootLocation(const sejong::Vect3 & com_pos,
                                   const sejong::Vect3 & com_vel,
                                   sejong::Vect3 & target_loc,
                                   const void* additional_input = NULL,
                                   void* additional_output = NULL);

  // Set Functions
  void setOmega(double com_height){
    b_set_omega_ = true;
    omega_ = sqrt(9.81/com_height);
  }
  void CheckEigenValues(double swing_time);

protected:
  std::vector<double> t_prime_;
  std::vector<double> kappa_;
  std::vector<double> x_step_length_limit_;
  std::vector<double> y_step_length_limit_;

  std::vector<double> com_vel_limit_;

  double omega_;
  bool b_set_omega_;

  void _computeSwitchingState(double swing_time,
                              const sejong::Vect3& com_pos,
                              const sejong::Vect3& com_vel,
                              const sejong::Vect3& stance_foot_loc,
                              std::vector<sejong::Vect2> & switching_state);
  void _StepLengthCheck(sejong::Vect3 & target_loc,
                        const std::vector<sejong::Vect2> & switching_state);
  void _StepLengthCheck(sejong::Vect3 & target_loc,
                        bool b_positive_sidestep,
                        const sejong::Vect3 & stance_foot);

  int _check_switch_velocity(const std::vector<sejong::Vect2> & switch_state);

};

#endif

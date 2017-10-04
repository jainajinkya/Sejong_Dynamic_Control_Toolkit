#ifndef LIPM_PLOT_SYSTEM
#define LIPM_PLOT_SYSTEM

#include "Plotting_System.h"
#include <Utils/wrap_eigen.hpp>
#include <vector>

class LIPM_3D;
class LIPM_Tester;

class LIPM_Plot_System: public Plotting_System{
public:
  LIPM_Plot_System(LIPM_3D* lipm,
                   const LIPM_Tester* tester);
  virtual ~LIPM_Plot_System();

  // 0: data count
  // 1: time
  // 2~4: CoM pos
  // 5~7: CoM vel
  // 8~10: pivot pos
  virtual void UpdateData(double time,
                          std::vector<double>& data);

  virtual double GetEndTime() { return end_time_; }
protected:
  double end_time_;
  int num_data_send_;
  int step_idx_;
  void _find_ini_state_curr_time(double global_time,
                                 sejong::Vector & state,
                                 sejong::Vector & pivot,
                                 double & local_time);

  LIPM_3D* lipm_;
  LIPM_Tester* tester_;

  std::vector<double> ini_time_list_;
  std::vector<sejong::Vector> ini_state_list_;
  std::vector<sejong::Vector> pivot_list_;
};

#endif

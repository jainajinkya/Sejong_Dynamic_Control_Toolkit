#ifndef PLOTTING_SYSTEM_OPENCHAIN_2D
#define PLOTTING_SYSTEM_OPENCHAIN_2D

#include <plotter/Plotting_System.hpp>

#if CTRL_MODEL == OPENCHAIN_2D
class PlotSys_Openchain2D: public Plotting_System{
public:
  PlotSys_Openchain2D();
  virtual ~PlotSys_Openchain2D();

  virtual void UpdateState(const std::vector<double>& config);

  virtual void GetStEndPt_Link(std::vector<double> & st_x,
                               std::vector<double> & st_y,
                               std::vector<double> & end_x,
                               std::vector<double> & end_y);

};

#endif
#endif

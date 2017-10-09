#ifndef PLOTTING_SYSTEM_OPENCHAIN_2DOF
#define PLOTTING_SYSTEM_OPENCHAIN_2DOF

#include <SimManager/Plotting_System.hpp>

class PlotSys_Openchain2DoF: public Plotting_System{
public:
  PlotSys_Openchain2DoF();
  virtual ~PlotSys_Openchain2DoF();

  virtual void GetStEndPt_Link(std::vector<double> & st_x,
                               std::vector<double> & st_y,
                               std::vector<double> & end_x,
                               std::vector<double> & end_y);

};

#endif

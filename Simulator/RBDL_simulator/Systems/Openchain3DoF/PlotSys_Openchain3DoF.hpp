#ifndef PLOTTING_SYSTEM_OPENCHAIN_3DOF
#define PLOTTING_SYSTEM_OPENCHAIN_3DOF

#include <SimManager/Plotting_System.hpp>

class PlotSys_Openchain3DoF: public Plotting_System{
public:
  PlotSys_Openchain3DoF();
  virtual ~PlotSys_Openchain3DoF();

  virtual void GetStEndPt_Link(std::vector<double> & st_x,
                               std::vector<double> & st_y,
                               std::vector<double> & end_x,
                               std::vector<double> & end_y);

};

#endif

#ifndef PLOTTING_SYSTEM_WALKER_2D
#define PLOTTING_SYSTEM_WALKER_2D

#include <SimManager/Plotting_System.hpp>

class PlotSys_Walker2D: public Plotting_System{
public:
  PlotSys_Walker2D();
  virtual ~PlotSys_Walker2D();

  virtual void GetStEndPt_Link(std::vector<double> & st_x,
                               std::vector<double> & st_y,
                               std::vector<double> & end_x,
                               std::vector<double> & end_y);

};

#endif

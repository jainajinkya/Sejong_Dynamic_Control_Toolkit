#ifndef PLOTTING_SYSTEM_FLOATING_LEG
#define PLOTTING_SYSTEM_FLOATING_LEG

#include <plotter/Plotting_System.hpp>

#if CTRL_MODEL == FLOATING_LEG

class PlotSys_FloatingLeg: public Plotting_System{
public:
  PlotSys_FloatingLeg();
  virtual ~PlotSys_FloatingLeg();

  virtual void UpdateState(const std::vector<double>& config);

  virtual void GetStEndPt_Link(std::vector<double> & st_x,
                               std::vector<double> & st_y,
                               std::vector<double> & end_x,
                               std::vector<double> & end_y);
  double m_mass_offset;
};

#endif
#endif

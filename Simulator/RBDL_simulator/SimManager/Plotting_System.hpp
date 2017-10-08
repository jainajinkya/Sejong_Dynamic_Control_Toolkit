#ifndef PLOTTING_SYSTEM
#define PLOTTING_SYSTEM

#include <vector>

class Plotting_System{
public:
  Plotting_System(){}
  virtual ~Plotting_System(){}

  virtual void GetStEndPt_Link(std::vector<double> & st_x,
                               std::vector<double> & st_y,
                               std::vector<double> & end_x,
                               std::vector<double> & end_y) = 0;
protected:

};

#endif

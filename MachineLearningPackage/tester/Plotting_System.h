#ifndef PLOTTING_SYSTEM
#define PLOTTING_SYSTEM

#include <vector>

#define NUM_VALUE_PYTHON 11

class Plotting_System{
public:
  Plotting_System(){}
  virtual ~Plotting_System(){}

  virtual void UpdateData(double time, std::vector<double> & data) = 0;
  virtual double GetEndTime() = 0;
protected:
};

#endif

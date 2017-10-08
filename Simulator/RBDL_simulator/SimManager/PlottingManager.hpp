#ifndef PLOTTING_MANAGER
#define PLOTTING_MANAGER

#include <iostream>
#include <Python.h>
#include <vector>

/* You don't need to touch this file
   to plot a new system */

class Plotting_System;

class PlottingManager{
public:
  PlottingManager();
  ~PlottingManager();

  // time & extra data
  void SendData(double time,
                const std::vector<double> & st_x,
                const std::vector<double> & st_y,
                const std::vector<double> & end_x,
                const std::vector<double> & end_y,
                const std::vector<double> & array);

protected:
  // Python
  PyObject *pName, *pModule, *pDict,
    *pFunc_send, *pFunc_update, *pArgs;

  std::vector<double> st_x, st_y, end_x, end_y;
  Plotting_System* plot_sys_;
};

#endif

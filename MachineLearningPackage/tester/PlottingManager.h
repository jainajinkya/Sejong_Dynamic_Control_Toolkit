#ifndef PLOTTING_MANAGER
#define PLOTTING_MANAGER

#include <iostream>
#include <Python.h>
#include <vector>

class Plotting_System;

class PlottingManager{
public:
  PlottingManager(Plotting_System* plot_sys);
  ~PlottingManager();

  void SendData(double time);

protected:
  // Python
  PyObject *pName, *pModule, *pDict,
    *pFunc_send, *pFunc_update, *pArgs;

  Plotting_System* plot_sys_;
};

#endif

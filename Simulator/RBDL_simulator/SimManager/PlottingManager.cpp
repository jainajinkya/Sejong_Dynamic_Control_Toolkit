#include "PlottingManager.hpp"
#include <RBDL_Sim_Configuration.h>
#include <Configuration.h>

/* You don't need to touch this file
   to plot a new system */

PlottingManager::PlottingManager():
  st_x(NUM_LINES, 0.),
  st_y(NUM_LINES, 0.),
  end_x(NUM_LINES, 0.),
  end_y(NUM_LINES, 0.)
{
  printf("Plotting Manager\n");
  Py_Initialize();

  std::string cmd_string( "import sys; sys.path.append('" );
  cmd_string.append( THIS_COM"/Simulator/RBDL_simulator/SimManager/')" );
  PyRun_SimpleString(cmd_string.c_str());

  // Now execute some python code (call python functions)
  pName = PyString_FromString("sender");
  pModule = PyImport_Import(pName);

  // pDict and pFunc are borrowed references
  pDict = PyModule_GetDict(pModule);
  pFunc_update = PyDict_GetItemString(pDict, "update");
  pFunc_send = PyDict_GetItemString(pDict, "send");
  bool b_python(PyCallable_Check(pFunc_send) && PyCallable_Check(pFunc_update));

  if(!b_python)   PyErr_Print();

  printf("Python Reading: %i \n", b_python);
}

void PlottingManager::SendData(double time,
                               const std::vector<double> & st_x,
                               const std::vector<double> & st_y,
                               const std::vector<double> & end_x,
                               const std::vector<double> & end_y,
                               const std::vector<double> & array){
  // printf("Data recieved : %f, ..., %f, %f, ...\n", array[0], array[NUM_Q-1], time);
  // NUM_Q
  // Others...
  pArgs = PyTuple_New(NUM_VALUE_PYTHON);
  // Link Start and End Position
  // i th Start End in X direction
  // i th Start End in Y direction
  // ...
  // Simulation time (sec)
  for( int k(0); k<NUM_LINES; ++k){
    PyTuple_SetItem(pArgs, 4*k, PyFloat_FromDouble(st_x[k]));
    PyTuple_SetItem(pArgs, 4*k+1, PyFloat_FromDouble(end_x[k]));

    PyTuple_SetItem(pArgs, 4*k+2, PyFloat_FromDouble(st_y[k]));
    PyTuple_SetItem(pArgs, 4*k+3, PyFloat_FromDouble(end_y[k]));
  }
  PyTuple_SetItem(pArgs, 4*NUM_LINES, PyFloat_FromDouble(time));

  int i(0);
  for(int k(4*NUM_LINES + 1); k<NUM_VALUE_PYTHON; ++k){
    PyTuple_SetItem(pArgs, k, PyFloat_FromDouble(array[i]));
    ++i;
  }
  PyObject_CallFunctionObjArgs(pFunc_update, pArgs, NULL);
  PyObject_CallObject(pFunc_send, NULL);
}

PlottingManager::~PlottingManager(){
  // Clean up
  Py_DECREF(pArgs);
  Py_DECREF(pModule);
  Py_DECREF(pName);

  Py_Finalize();
}

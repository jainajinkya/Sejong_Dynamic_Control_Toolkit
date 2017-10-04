#include "PlottingManager.h"
#include "Plotting_System.h"

#define THIS_COM "/Users/donghyunkim/locomotion_controller/C_Learning_Study/"

PlottingManager::PlottingManager(Plotting_System* plot_sys):
  plot_sys_(plot_sys)
{

  Py_Initialize();

  std::string cmd_string( "import sys; sys.path.append('" );
  cmd_string.append( THIS_COM"tester/python/')" );
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

void PlottingManager::SendData(double time){
  std::vector<double> data;
  plot_sys_->UpdateData(time, data);
  pArgs = PyTuple_New(NUM_VALUE_PYTHON);

  for( int k(0); k<NUM_VALUE_PYTHON; ++k){
    PyTuple_SetItem(pArgs, k, PyFloat_FromDouble(data[k]));
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

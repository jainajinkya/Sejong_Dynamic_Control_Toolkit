#include "PlotSys_Openchain2DoF.hpp"
#include "OC2_Sim_Model.hpp"
#include <RBDL_Sim_Configuration.h>

PlotSys_Openchain2DoF::PlotSys_Openchain2DoF():Plotting_System(){}
PlotSys_Openchain2DoF::~PlotSys_Openchain2DoF(){}


void PlotSys_Openchain2DoF::GetStEndPt_Link(std::vector<double> & st_x,
                                          std::vector<double> & st_y,
                                          std::vector<double> & end_x,
                                          std::vector<double> & end_y){
  OC2_Sim_Model* model = OC2_Sim_Model::GetOC2_Sim_Model();

  st_x.resize(NUM_LINES);
  st_y.resize(NUM_LINES);

  end_x.resize(NUM_LINES);
  end_y.resize(NUM_LINES);

  Eigen::Vector3d pos;

  st_x[0] = 0.;
  st_y[0] = 0.;

  // L1
  model->getPos(SJ_SIM_LinkID::LK_SIM_J2, pos);
  end_x[0] = pos[0];
  end_y[0] = pos[1];

  st_x[1] = pos[0];
  st_y[1] = pos[1];

  // EE
  model->getPos(SJ_SIM_LinkID::LK_SIM_EE, pos);

  end_x[1] = pos[0];
  end_y[1] = pos[1];
}

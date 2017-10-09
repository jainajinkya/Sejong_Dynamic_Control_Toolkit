#include "PlotSys_Openchain3DoF.hpp"
#include "OC3_Sim_Model.hpp"
#include <RBDL_Sim_Configuration.h>

PlotSys_Openchain3DoF::PlotSys_Openchain3DoF():Plotting_System(){}
PlotSys_Openchain3DoF::~PlotSys_Openchain3DoF(){}


void PlotSys_Openchain3DoF::GetStEndPt_Link(std::vector<double> & st_x,
                                          std::vector<double> & st_y,
                                          std::vector<double> & end_x,
                                          std::vector<double> & end_y){
  OC3_Sim_Model* model = OC3_Sim_Model::GetOC3_Sim_Model();

  st_x.resize(NUM_LINES);
  st_y.resize(NUM_LINES);

  end_x.resize(NUM_LINES);
  end_y.resize(NUM_LINES);

  Eigen::Vector3d pos;

  // L1
  model->getPos(SJ_SIM_LinkID::LK_SIM_J2, pos);
  st_x[0] = 0.;
  st_y[0] = 0.;
  end_x[0] = pos[0];
  end_y[0] = pos[1];

  st_x[1] = pos[0];
  st_y[1] = pos[1];

  // Thigh
  model->getPos(SJ_SIM_LinkID::LK_SIM_J3, pos);

  end_x[1] = pos[0];
  end_y[1] = pos[1];

  st_x[2] = pos[0];
  st_y[2] = pos[1];

  //Hip
  model->getPos(SJ_SIM_LinkID::LK_SIM_EE, pos);

  end_x[2] = pos[0];
  end_y[2] = pos[1];
}

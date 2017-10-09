#include "PlotSys_Walker2D.hpp"
#include "Walker2D_Sim_Model.hpp"
#include <RBDL_Sim_Configuration.h>

PlotSys_Walker2D::PlotSys_Walker2D():Plotting_System(){}
PlotSys_Walker2D::~PlotSys_Walker2D(){}


void PlotSys_Walker2D::GetStEndPt_Link(std::vector<double> & st_x,
                                          std::vector<double> & st_y,
                                          std::vector<double> & end_x,
                                          std::vector<double> & end_y){
  Walker2D_Sim_Model* model = Walker2D_Sim_Model::GetWalker2D_Sim_Model();

  st_x.resize(NUM_LINES);
  st_y.resize(NUM_LINES);

  end_x.resize(NUM_LINES);
  end_y.resize(NUM_LINES);

  Eigen::Vector3d pos, hip_pos;


  model->getPos(SJ_SIM_LinkID::LK_SIM_HIP, hip_pos);
  // Body
  st_x[0] = hip_pos[0];
  st_y[0] = hip_pos[1];

  model->getPos(SJ_SIM_LinkID::LK_SIM_BODY_EE, pos);
  end_x[0] = pos[0];
  end_y[0] = pos[1];

  // Left Thight
  st_x[1] = hip_pos[0];
  st_y[1] = hip_pos[1];

  model->getPos(SJ_SIM_LinkID::LK_SIM_LEFT_KNEE, pos);
  end_x[1] = pos[0];
  end_y[1] = pos[1];

  // Left Shank
  st_x[2] = pos[0];
  st_y[2] = pos[1];

  model->getPos(SJ_SIM_LinkID::LK_SIM_LEFT_FOOT, pos);
  end_x[2] = pos[0];
  end_y[2] = pos[1];

  // Right Thight
  st_x[3] = hip_pos[0];
  st_y[3] = hip_pos[1];

  model->getPos(SJ_SIM_LinkID::LK_SIM_RIGHT_KNEE, pos);
  end_x[3] = pos[0];
  end_y[3] = pos[1];

  // Right Shank
  st_x[4] = pos[0];
  st_y[4] = pos[1];

  model->getPos(SJ_SIM_LinkID::LK_SIM_RIGHT_FOOT, pos);
  end_x[4] = pos[0];
  end_y[4] = pos[1];
}

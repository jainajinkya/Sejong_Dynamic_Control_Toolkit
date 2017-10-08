#include <plotter/PlotSys_Openchain2D.hpp>
#include <model/Openchain2D.hpp>
#include <plotter/Plotter_Configuration.h>

#if CTRL_MODEL == OPENCHAIN_2D

PlotSys_Openchain2D::PlotSys_Openchain2D():Plotting_System(){
  
}

PlotSys_Openchain2D::~PlotSys_Openchain2D(){
  
}


void PlotSys_Openchain2D::UpdateState(const std::vector<double> & config){
  
}

void PlotSys_Openchain2D::GetStEndPt_Link(std::vector<double> & st_x,
                                          std::vector<double> & st_y,
                                          std::vector<double> & end_x,
                                          std::vector<double> & end_y){
  RobotModel* model = RobotModel::getRobotModel();

  st_x.resize(NUM_LINK);
  st_y.resize(NUM_LINK);

  end_x.resize(NUM_LINK);
  end_y.resize(NUM_LINK);

  Vector3d pos;

  // Shank
  model->getPos(0, pos);

  st_x[0] = 0.;
  st_y[0] = 0.;
  end_x[0] = pos[0];
  end_y[0] = pos[1];

  st_x[1] = pos[0];
  st_y[1] = pos[1];

  // Thigh
  model->getPos(1, pos);

  end_x[1] = pos[0];
  end_y[1] = pos[1];
}
#endif

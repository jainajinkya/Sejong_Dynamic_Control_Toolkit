#include <plotter/PlotSys_Openchain3D.hpp>
#include <model/RobotModel.hpp>
#include <model/FloatingLeg.hpp>
#include <plotter/Plotter_Configuration.h>

#if CTRL_MODEL == FLOATING_LEG

PlotSys_FloatingLeg::PlotSys_FloatingLeg():Plotting_System(),
                                           m_mass_offset(0.05){

}

PlotSys_FloatingLeg::~PlotSys_FloatingLeg(){

}


void PlotSys_FloatingLeg::UpdateState(const std::vector<double> & config){

}

void PlotSys_FloatingLeg::GetStEndPt_Link(std::vector<double> & st_x,
                                          std::vector<double> & st_y,
                                          std::vector<double> & end_x,
                                          std::vector<double> & end_y){
  RobotModel* model = RobotModel::getRobotModel();

  st_x.resize(NUM_LINES);
  st_y.resize(NUM_LINES);

  end_x.resize(NUM_LINES);
  end_y.resize(NUM_LINES);

  Eigen::Vector3d pos;

  // Body offset
  model->getPos(LinkID::LK_BODY, pos);
  st_x[0] = pos[0];
  st_y[0] = pos[1] + m_mass_offset;

  model->getPos(LinkID::LK_THIGH, pos);
  end_x[0] = pos[0];
  end_y[0] = pos[1];

  // Thigh
  st_x[1] = pos[0];
  st_y[1] = pos[1];

  model->getPos(LinkID::LK_SHANK, pos);

  end_x[1] = pos[0];
  end_y[1] = pos[1];

  // Shank
  st_x[2] = pos[0];
  st_y[2] = pos[1];

  model->getPos(LinkID::LK_FOOT, pos);

  // end_x[2] = 0.;//pos[0];
  // end_y[2] = 0.;//pos[1];

  // // Foot
  // st_x[3] = 0.;//pos[0];
  // st_y[3] = 0.;//pos[1];
  end_x[2] = pos[0];
  end_y[2] = pos[1];

  // Foot
  st_x[3] = pos[0];
  st_y[3] = pos[1];

  model->getPos(LinkID::LK_TOE, pos);

  end_x[3] = pos[0];
  end_y[3] = pos[1];

  // Toe to Heel
  st_x[4] = pos[0];
  st_y[4] = pos[1];

  model->getPos(LinkID::LK_HEEL, pos);

  end_x[4] = pos[0];
  end_y[4] = pos[1];

  // Heel to Ankle
  st_x[5] = pos[0];
  st_y[5] = pos[1];

  model->getPos(LinkID::LK_FOOT, pos);

  end_x[5] = pos[0];
  end_y[5] = pos[1];

  // Mass Center
  model->getPos(LinkID::LK_MASS_CENTER, pos);
  st_x[6] = pos[0];
  st_y[6] = pos[1];
  end_x[6] = pos[0];
  end_y[6] = pos[1];
}

#endif

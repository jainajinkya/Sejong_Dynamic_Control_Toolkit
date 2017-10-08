#include <plotter/FloatingLegTest.hpp>
#include <ctrl/TestControl.hpp>
#include <utils/ParameterFetcher.hpp>

#if CTRL_MODEL == FLOATING_LEG

FloatingLegTest::FloatingLegTest():CtrlTester(){
  ros::NodeHandle nh("~");
  std::string controller_name;

  ParameterFetcher::getReqParam(nh,"controller/name",controller_name);

  if(controller_name.compare("DemoControl_3D") == 0){
    ROS_INFO("DemoControl initialized.");
    // m_ctrl = new DemoControl_3D(nh);
  } else if (controller_name.compare("TestControl") == 0){
    m_ctrl = new TestControl(nh);
  } else {
    ROS_ERROR("Controller name, '%s', not recognized. ", controller_name.c_str());
    ros::shutdown();
    exit(1);
  }
}

void FloatingLegTest::Initialization(){
  m_jpos_ini.setZero();
  m_jvel_ini.setZero();
  m_jeff_ini.setZero();

  m_jpos_ini[0] = -0.9; 
  m_jpos_ini[1] = M_PI/2. - 0.3;
  m_jpos_ini[2] = -m_jpos_ini[0] - m_jpos_ini[1] - M_PI/2.;

}

void FloatingLegTest::_ExtraProcess(){
}

void FloatingLegTest::_DataPrint(){
}

#endif

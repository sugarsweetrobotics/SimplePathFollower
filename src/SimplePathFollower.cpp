// -*- C++ -*-
/*!
 * @file  SimplePathFollower.cpp
 * @brief Simple Algorithm Path Follower
 * @date $Date$
 *
 * $Id$
 */

#include "SimplePathFollower.h"

// Module specification
// <rtc-template block="module_spec">
static const char* simplepathfollower_spec[] =
  {
    "implementation_id", "SimplePathFollower",
    "type_name",         "SimplePathFollower",
    "description",       "Simple Algorithm Path Follower",
    "version",           "1.0.0",
    "vendor",            "Sugar Sweet Robotics",
    "category",          "Experimenta",
    "activity_type",     "PERIODIC",
    "kind",              "DataFlowComponent",
    "max_instance",      "1",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    "conf.default.debug", "0",
    "conf.default.directionGain", "1.0",
    "conf.default.distanceGain", "1.0",
    // Widget
    "conf.__widget__.debug", "text",
    "conf.__widget__.directionGain", "text",
    "conf.__widget__.distanceGain", "text",
    // Constraints
    ""
  };
// </rtc-template>

/*!
 * @brief constructor
 * @param manager Maneger Object
 */
SimplePathFollower::SimplePathFollower(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_pathIn("path", m_path),
    m_currentPoseIn("currentPose", m_currentPose),
    m_velocityOut("velocity", m_velocity),
    m_PathFollowerPort("PathFollower")

    // </rtc-template>
{
}

/*!
 * @brief destructor
 */
SimplePathFollower::~SimplePathFollower()
{
}



RTC::ReturnCode_t SimplePathFollower::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("path", m_pathIn);
  addInPort("currentPose", m_currentPoseIn);
  
  // Set OutPort buffer
  addOutPort("velocity", m_velocityOut);
  
  // Set service provider to Ports
  m_PathFollowerPort.registerProvider("PathFollower", "RTC::PathFollower", m_pathFollower);
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  addPort(m_PathFollowerPort);
  
  // </rtc-template>

  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  bindParameter("debug", m_debug, "0");
  bindParameter("directionGain", m_directionGain, "1.0");
  bindParameter("distanceGain", m_distanceGain, "1.0");
  // </rtc-template>
  
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t SimplePathFollower::onFinalize()
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t SimplePathFollower::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t SimplePathFollower::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/


RTC::ReturnCode_t SimplePathFollower::onActivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}


RTC::ReturnCode_t SimplePathFollower::onDeactivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}


RTC::ReturnCode_t SimplePathFollower::onExecute(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t SimplePathFollower::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t SimplePathFollower::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t SimplePathFollower::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t SimplePathFollower::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t SimplePathFollower::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/



extern "C"
{
 
  void SimplePathFollowerInit(RTC::Manager* manager)
  {
    coil::Properties profile(simplepathfollower_spec);
    manager->registerFactory(profile,
                             RTC::Create<SimplePathFollower>,
                             RTC::Delete<SimplePathFollower>);
  }
  
};



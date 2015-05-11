// -*-C++-*-
/*!
 * @file  MobileRobotSVC_impl.cpp
 * @brief Service implementation code of MobileRobot.idl
 *
 */

#include "MobileRobotSVC_impl.h"

#include "SimplePathFollower.h"

/*
 * Example implementational code for IDL interface RTC::PathFollower
 */
PathFollowerSVC_impl::PathFollowerSVC_impl()
{
  // Please add extra constructor code here.
}


PathFollowerSVC_impl::~PathFollowerSVC_impl()
{
  // Please add extra destructor code here.
}


/*
 * Methods corresponding to IDL attributes and operations
 */
RTC::RETURN_VALUE PathFollowerSVC_impl::followPath(const RTC::Path2D& path)
{
  RTC::RETURN_VALUE result = RETVAL_OK;
  // Please insert your code here and remove the following warning pragma

  std::cout << "[RTC::SimplePathFollower] Start Following" << std::endl;
  
  m_pRTC->setPath(path);
  m_pRTC->startFollow();

  while(!m_pRTC->isGoal()) {
    coil::usleep(1000*100);

    if (m_pRTC->getMode() == MODE_TIMEOUT) {
        std::cout << "[RTC::SimplePathFollower] Current Pose Timeout" << std::endl;
        return RTC::RETURN_VALUE::RETVAL_CURRENT_POSE_TIME_OUT;
    }
  }
 
  std::cout << "[RTC::SimplePathFollower] Goal Reached." << std::endl;
  
  return result;
}



// End of example implementational code




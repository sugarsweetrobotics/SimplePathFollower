// -*-C++-*-
/*!
 * @file  MobileRobotSVC_impl.cpp
 * @brief Service implementation code of MobileRobot.idl
 *
 */

#include "MobileRobotSVC_impl.h"

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
	RTC::RETURN_VALUE result;
  // Please insert your code here and remove the following warning pragma
#ifndef WIN32
  #warning "Code missing in function <RTC::RETURN_VALUE PathFollowerSVC_impl::followPath(const RTC::Path2D& path)>"
#endif
  return result;
}



// End of example implementational code




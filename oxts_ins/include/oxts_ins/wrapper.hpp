/**
 * \file ros_ncom_wrapper.hpp
 * Functions to wrap NCom data in ROS messages
 */

#ifndef ROS_NCOM_WRAPPER
#define ROS_NCOM_WRAPPER

// Standard includes
#include <string>
#include <vector>
#include <cmath>
#include <iostream>

// ROS message types includes
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/nav_sat_status.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/time_reference.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <geometry_msgs/msg/quaternion.h>
#include <geometry_msgs/msg/point.h>
#include <tf2/LinearMath/Quaternion.h>
#include <oxts_msgs/msg/nav_sat_ref.hpp>

#include "tf2_kdl/tf2_kdl.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

// OxTS includes
#include "oxts_ins/NComRxC.h"
#include "oxts_ins/nav_const.hpp"
#include "oxts_ins/nav_conversions.hpp"

/**
 * Functions to convert data from the NCom decoder to ROS messages
 */
namespace RosNComWrapper
{


  /**
   * Calculate the rotational component of the transform from frame1 to frame2
   * based on Euler angles
   * 
   * @param nrx Pointer to the decoded NCom data
   * @return A quaternion representing the rotation between vehicle frame and imu frame
   */
  tf2::Quaternion getVat(const NComRxC *nrx);
  /**
   * Calculate the lateral no slip component of the transform from frame1 to frame2
   * 
   * @param nrx Pointer to the decoded NCom data
   * @return A transform representing the translation between the vehicle rear axle and the imu frame
   */
  tf2::Vector3 getNsp(const NComRxC *nrx);
  /**
   * Get the NCOM orientation in body (IMU) frame
   * 
   * @param nrx Pointer to the decoded NCom data
   * @return A quaterntion representing the rotation between the vehicle frame and NED
   */
  tf2::Quaternion getBodyRPY(const NComRxC *nrx);
  /**
   * Get the NCOM orientation in vehicle frame
   * 
   * @param nrx Pointer to the decoded NCom data
   * @return A quaterntion representing the rotation between the vehicle frame and NED
   */
  tf2::Quaternion getVehRPY(const NComRxC *nrx);
  /**
   * Get the LRF from the NCOM decoder
   */
  Lrf getNcomLrf(const NComRxC *nrx);
  /** 
   * Wrap data from NCom decoder to std_msgs/msg/NavSatStatus
   * 
   * @param nrx Pointer to the decoded NCom data
   */
  sensor_msgs::msg::NavSatStatus     nav_sat_status(const NComRxC *nrx);
  /**
   * Wrap data from NCom decoder to sensor_msgs/msg/NavSatFix
   * 
   * @param nrx Pointer to the decoded NCom data
   * @param head Header to be added to the published message
   */
  sensor_msgs::msg::NavSatFix        nav_sat_fix(const NComRxC *nrx,
                                                    std_msgs::msg::Header head);
  /**
   * Wrap data for local reference point using oxts_msgs/msg/NavSatRef
   * 
   * @param nrx Pointer to the decoded NCom data
   * @param head Header to be added to the published message
   */
  oxts_msgs::msg::NavSatRef        nav_sat_ref(const NComRxC *nrx,
                                                    std_msgs::msg::Header head);

  /**
   * Wrap data from the NCom decoder to 
   * geometry_msgs/msg/PointStamped. 
   * 
   * @param nrx Pointer to the decoded NCom data.
   * @param head Header to be added to the published message.
   * @return Position of the IMU in the ECEF coordinate frame.
   */
  geometry_msgs::msg::PointStamped ecef_pos
                                                (
                                                const NComRxC *nrx,
                                                std_msgs::msg::Header head
                                                );
  /**
   * Wrap position data from NCom decoder to std_msgs/msg/String
   * Not really a permanent function, more for easy testing.
   * 
   * @param nrx Pointer to the decoded NCom data
   */
  std_msgs::msg::String              string   (const NComRxC *nrx); 
  /**
   * Wrap IMU data from NCom decoder to sensor_msgs/msg/Imu
   * 
   * @param nrx Pointer to the decoded NCom data
   * @param head Header to be added to the published message
   * 
   * \todo Covariances
   */
  sensor_msgs::msg::Imu              imu      (const NComRxC *nrx,
                                                    std_msgs::msg::Header head);
  /**
   * Wrap velocity data from NCom decoder to sensor_msgs/msg/Imu
   * 
   * @param nrx Pointer to the decoded NCom data
   * @param head Header to be added to the published message
   * 
   * @returns Linear velocity forward, lateral, down. 
   *          Angular velocity forward, lateral, down.
   */
  geometry_msgs::msg::TwistStamped   velocity (const NComRxC *nrx,
                                                    std_msgs::msg::Header head);
  /**
   * Wrap navigation data from NCom decoder to nav_msgs/msg/Odometry
   * 
   * @param nrx Pointer to the decoded NCom data
   * @param head Header to be added to the published message
   * @returns 
   */
  nav_msgs::msg::Odometry odometry (const NComRxC *nrx,
                                    std_msgs::msg::Header head,
                                    Lrf lrf);
  /**
   * Wrap time data from NCom decoder to sensor_msgs/msg/TimeReference
   * 
   * The time reference depends on the presence of GNSS signal.
   * 
   * @param nrx Pointer to the decoded NCom data
   * @param head Header to be added to the published message
   * 
   */
  sensor_msgs::msg::TimeReference time_reference (const NComRxC *nrx,
                                                    std_msgs::msg::Header head);
  /**
   * Wrap tf data from NCom decoder to sensor_msgs/msg/TimeReference
   * 
   * Transform from global ECEF to IMU
   * 
   * @param nrx Pointer to the decoded NCom data
   * @param head Header to be added to the published message
   * 
   */
  geometry_msgs::msg::TransformStamped tf2    (const NComRxC *nrx,
                                                    std_msgs::msg::Header head);
}




#endif //ROS_NCOM_WRAPPER
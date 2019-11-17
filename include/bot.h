/**
 * @file bot.h
 * @brief C++ header file for the Walkerbot class
 *
 *
 * @copyright Copyright (c) Fall 2019 ENPM808X
 *            This project is released under the BSD 3-Clause License.
 *
 * @author Chinmay Joshi
 *
 * @date 11-16-2019
 */

#ifndef _HOME_CHINMAY17_CATKIN_WS_SRC_TURTLEBOT_ENPM808X_INCLUDE_BOT_H_
#define _HOME_CHINMAY17_CATKIN_WS_SRC_TURTLEBOT_ENPM808X_INCLUDE_BOT_H_

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

/**
 * @brief Waler class for the turtlebot robot which takes in the
 *        LaserScan data
 */
class roomba {
 private:
  // private member variable to store the obstacle state.
  static bool obs;

 public:
/**
 * @brief This is the first method of the class. It is a function
 *        for subscribing to the laser scan data.
 *
 * @param Message published by the LaserScan node of turtlebot as input.
 */
  void readDistance(const sensor_msgs::LaserScan::ConstPtr&);

/**
 * @brief This is the second method of the class. It is a function
 *        which returns if an obstacle is present or not.
 *
 * @return Returns if an obstacle is detected or not.
 */
  bool obstacle();
};

#endif  // _HOME_CHINMAY17_CATKIN_WS_SRC_TURTLEBOT_ENPM808X_INCLUDE_BOT_H_

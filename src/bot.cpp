/**
 * @file bot.cpp
 * @brief C++ implementation file for the Walkerbot class
 *
 *
 * @copyright Copyright (c) Fall 2019 ENPM808X
 *            This project is released under the BSD 3-Clause License.
 *
 * @author Chinmay Joshi
 *
 * @date 11-16-2019
 */

#include <ros/ros.h>
#include <bot.h>

// Assigning the value of obs to false in the start
bool roomba::obs = false;

/**
 * @brief This is the first method of the class. It is a function
 *        for subscribing to the laser scan data.
 *
 * @param Message published by the LaserScan node of turtlebot as input.
 */
void roomba::readDistance(const sensor_msgs::LaserScan::ConstPtr& val) {
  for (auto j : val -> ranges) {
    if (j < 1.0) {
    // setting value to true if there is an obstacle present
    obs = true;
    return;
    }
  }
  // setting false if no obstacle is present
  obs = false;
}

/**
 * @brief This is the second method of the class. It is a function
 *        which returns if an obstacle is present or not.
 *
 * @return Returns if an obstacle is detected or not.
 */
bool roomba::obstacle() {
  return obs;
}

/**
 * @file main.cpp
 * @brief C++ implementation file for the roomba robot
 *
 *
 * @copyright Copyright (c) Fall 2019 ENPM808X
 *            This project is released under the BSD 3-Clause License.
 *
 * @author Chinmay Joshi
 *
 * @date 11-16-2019
 */

#include <sensor_msgs/LaserScan.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <bot.h>

/**
 * This tutorial demonstrates simple obstacle avoidance using turtlebot
 */
int main(int argc, char **argv) {
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "roomba");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  // Creating an object of class roomba which will be used for
  // obstacle avoidance
  roomba walker;

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
  auto laserSensor = n.subscribe<sensor_msgs::LaserScan>("/scan", 50,
                                             &roomba::readDistance, &walker);

  auto vel = n.advertise<geometry_msgs::Twist>
                                    ("/mobile_base/commands/velocity", 1000);

  ros::Rate loop_rate(10);

  geometry_msgs::Twist msg;

  // Adding an intial angle because the orientation of turtlebot was such
  // that it would have never reached an obstacle
  msg.angular.z = -0.5;

  /**
   * A count of how many messages we have sent.
   */

  int count = 0;
  while (ros::ok()) {
     // If no obstacle is present
     if (!walker.obstacle()) {
       // Move forward
       msg.linear.x = 0.5;

       // robot should not rotate
       msg.angular.z = 0.0;

       ROS_INFO_STREAM("Going Forward");

     } else {
       // If obstacle is present
       // Rotate on axis to avoid obstacle
       msg.angular.z = -0.5;

       // Robot should not move forward while rotating
       msg.linear.x = 0.0;

       ROS_INFO_STREAM("Obstacle detected! Turning to avoid it");
     }

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    vel.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }

  return 0;
}

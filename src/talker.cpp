/************************************************************************
 MIT License
 Copyright (c) 2018 Ajeet Wankhede
 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:
 The above copyright notice and this permission notice shall be included in all
 copies or substantial portions of the Software.
 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 SOFTWARE.
 *************************************************************************/

/**
 *  @copyright MIT License 2018 Ajeet Wankhede
 *  @file    talker.cpp
 *  @author  Ajeet Wankhede
 *  @date    11/12/2018
 *  @version 1.0
 *
 *  @brief UMD ENPM 808X, Week 8, ROS Publisher/Subscriber
 *
 *  @section DESCRIPTION
 *
 *  Beginner tutorial for creating a ROS package to publish custom string message
 *  This tutorial demonstrates simple publishing of messages over the ROS system.
 *  This node broadcasts a tf frame /talk with /world
 */
#include <log4cxx/logger.h>
#include <std_srvs/Empty.h>
#include <sstream>
#include "ros/ros.h"
#include <tf/transform_broadcaster.h>
#include "std_msgs/String.h"
#include "beginner_tutorials/change_text.h"

// Create a struct containing the text message
struct text {
  std::string message;
} t;

/**
 * This is a message object. You stuff it with data, and then publish it.
 */
//std::string message = "First ROS package ";

/**
 *   @brief Service for changing the text message
 *
 *   @param req: request by client
 *   @param res: request of the server
 *
 *   @return bool
 */
bool changeText(beginner_tutorials::change_text::Request& req,
                beginner_tutorials::change_text::Response& res) {
  t.message = req.newString;
  res.respString = t.message;
  return true;
}

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
  ros::init(argc, argv, "talker");
  // Change the logging level of this node to Debug
  log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME)->setLevel(
      ros::console::g_level_lookup[ros::console::levels::Debug]);
  ros::console::notifyLoggerLevelsChanged();
  double frequency = 10;
  // Check if any argument is passed
  if (argc == 2) {
    frequency = atoi(argv[1]);
    // Check if the frequency is less than zero
    if (frequency < 0) {
      ROS_FATAL_STREAM("The frequency needs to be a positive real number.");
      return -1;
    }
    // Check is the frequency is equal to zero
    if (frequency == 0) {
      ROS_WARN_STREAM("The frequency is 0. It should be greater than zero.");
      return -1;
    }
  }
  // Print the set frequency
  ROS_DEBUG_STREAM(
      "The frequency of the talker node is set to " << frequency << ".");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  // Register our service with the master
  ros::ServiceServer server = n.advertiseService("change_text", changeText);

  // Create a TransformBroadcaster object which will be used to 
  // boardcast the transformation
  static tf::TransformBroadcaster br;
  // Create a Transform object
  tf::Transform transform;
  // Set translation and rotation for the talk frame with respect to world frame
  transform.setOrigin( tf::Vector3(2.0, 5.0, 0.0) );
  tf::Quaternion q;
  q.setRPY(0, 0, 1.57);
  transform.setRotation(q);

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
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

  ros::Rate loop_rate(frequency);

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;
  t.message = "First ROS package ";
  while (ros::ok()) {
    // Sending the transform
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "talk"));
    // Check if the message is empty
    if (t.message == "") {
      ROS_ERROR_STREAM("The message is empty.");
    }

    std::stringstream ss;
    ss << t.message << count;
    std_msgs::String msg;
    msg.data = ss.str();

    ROS_INFO_STREAM("" << msg.data.c_str());

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */

    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }
  return 0;
}

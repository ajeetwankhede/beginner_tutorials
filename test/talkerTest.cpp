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
 *  @file    talkerTest.cpp
 *  @author  Ajeet Wankhede
 *  @date    11/13/2018
 *  @version 1.0
 *
 *  @brief UMD ENPM 808X, Week 11, ROS Publisher/Subscriber ROS TF, unit testing, bag files
 *
 *  @section DESCRIPTION
 *
 *  This is a test source file for testing talker node
 */

#include <ros/ros.h>
#include <gtest/gtest.h>
#include <ros/service_client.h>
#include "std_msgs/String.h"
#include "beginner_tutorials/change_text.h"

/**
 * @brief This is a test to check the existence of ROS service change_text 
 * provided by talker node
 */
TEST(TalkerTest, serviceExistenceTest){
  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;
  // Setup a client to the service
  ros::ServiceClient client =
      n.serviceClient<beginner_tutorials::change_text>("change_text");
  // Check if the service created by talker node exists
  bool exists(client.waitForExistence(ros::Duration(10)));
  EXPECT_TRUE(exists);
}

/**
 * @brief This is a test to check if the service works correctly
 */
TEST(TalkerTest, serviceTest) {
  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;
  // Setup a client to request the service
  ros::ServiceClient client =
      n.serviceClient<beginner_tutorials::change_text>("change_text");
  // Create a service object
  /*beginner_tutorials::change_text srv;
  // Assign the text that needs to be changed
  srv.request.newString = "Testing change_text service";
  // Call the service
  client.call(srv);
  // Check the response
  EXPECT_STREQ("Testing change_text service", srv.response.respString.c_str());
  */
  beginner_tutorials::change_text::Request req;
  beginner_tutorials::change_text::Response resp;

  req.newString = "Testing change_text service";
  std::string expectedString = req.newString;

  bool success = client.call(req, resp);
  EXPECT_TRUE(success);
  EXPECT_EQ(expectedString, resp.respString);
}

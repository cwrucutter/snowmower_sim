/******************************************************************************
sensor_sim.cpp
The SenseSim class moniters the robot's state and simulates IMU and encoder
measurements.
*******************************************************************************
The MIT License (MIT)

  Copyright (c) 2016 Matthew A. Klein

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
******************************************************************************/

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <snowmower_msgs/EncMsg.h>
#include <nav_msgs/Odometry.h>
#include "snowmower_sim/sensor_sim.h"

/* odomSubCB - Odom callback function *
 * When an nav_msgs::Odometry message arrives on the topic odom, simulate an
 * IMU and encoder measurement and publish them.
 */
void SenseSim::odomSubCB(const nav_msgs::Odometry& msg){
  double dtEnc = dtEncoder(msg.header.stamp);
  ROS_INFO_STREAM("dtEnc = " << dtEnc);
  snowmower_msgs::EncMsg enc_msg;
  sensor_msgs::Imu imu_msg;

  // Simulate sensors!
  ROS_INFO_STREAM("Odom message recieved. Simulating sensors.");

  // Store current encoder measurement as enc_pre_
  enc_pre_ = enc_msg;
}

/* odomSubCB - Odom callback function *
 * When an nav_msgs::Odometry message arrives on the topic odom, simulate an
 * IMU and encoder measurement and publish them.
 */
void SenseSim::odomMapSubCB(const nav_msgs::Odometry& msg){
  double dtEnc = dtEncoder(msg.header.stamp);
  ROS_INFO_STREAM("dtEnc = " << dtEnc);
  snowmower_msgs::EncMsg enc_msg;
  sensor_msgs::Imu imu_msg;

  // Simulate sensors!
  ROS_INFO_STREAM("Odom message recieved. Simulating sensors.");

  // Store current encoder measurement as enc_pre_
  enc_pre_ = enc_msg;
}

// Determine time since the last time dtEnc() was called.
double SenseSim::dtEncoder(ros::Time currentEncTime){
  ros::Duration dtEnc;
  dtEnc = currentEncTime - lastEncTime_;
  lastEncTime_ = currentEncTime;
  return dtEnc.toSec();
}


/* Constructor */
SenseSim::SenseSim(): private_nh_("~") {

  // Listen for odometry updates on the following topics. Updates on odom are
  // used to simulate Imu and wheel encoders. Updates on odom_map are used to
  // simulate decawave and LIDAR (not used, currently).
  odomSub_ = public_nh_.subscribe("odom",1,&SenseSim::odomSubCB,this);
  odomMapSub_ = public_nh_.subscribe("odom_map",1,&SenseSim::odomMapSubCB,this);

  encPub_ = public_nh_.advertise<snowmower_msgs::EncMsg>("enc",1);
  imuPub_ = public_nh_.advertise<sensor_msgs::Imu>("imu/data",1);

  // Wait for time to not equal zero. A zero time means that no message has
  // been received on the /clock topic
  ros::Time timeZero(0.0);
  while (ros::Time::now() == timeZero) { }
  // Sleep for a small time to make sure publishing and subscribing works.
  ros::Duration(0.1).sleep();
  // Initialize 
  // init();
}

/* Destructor */
SenseSim::~SenseSim() {

}

int main(int argc, char **argv) {
  // Initialize ROS
  ros::init(argc, argv, "sense_sim");
  // Create an EkfNode Object
  SenseSim senseSim;
  // And spin
  ros::spin();

  return 0;
}

/******************************************************************************
sensor_sim.h
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

#ifndef __SENSOR_SIM_H_INCLUDED__
#define __SENSOR_SIM_H_INCLUDED__

class SenseSim {

 private:
  // Node specific stuff
  ros::NodeHandle public_nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber odomSub_;
  ros::Subscriber odomMapSub_;
  ros::Publisher encPub_;
  ros::Publisher imuPub_;

  // Odometry callback functions
  void odomSubCB(const nav_msgs::Odometry& msg);
  void odomMapSubCB(const nav_msgs::Odometry& msg);

  std::string base_frame_; // Frame of the robot
  std::string odom_frame_; // Frame of odom
  std::string map_frame_;  // Frame of the map

  // Sore last odom message to calculate new encoder reading (maybe?).
  nav_msgs::Odometry odom_pre_;

  // Wheel Track
  double b_;
  // Left and right encoder ticks per meter traveled
  int tpmRight_;
  int tpmLeft_;
  // Standard Deviation of sensors
  double enc_std_;
  double imu_std_;
  double dw_std_;

  // Sore last Encoder tick count. Needed to simulate new encoder reading.
  snowmower_msgs::EncMsg enc_pre_;
  // Also need the time of the last Encoder update.
  ros::Time lastEncTime_;
  // Determine time since the last time dtEncoder() was called.
  double dtEncoder(ros::Time currentEncTime);

  // Variables needed to create normally distributed random numbers
  float fn_[128];
  float wn_[128];
  uint32_t kn_[128];
  uint32_t seed_;

  // Initialize the parameters
  void init();

 public:
  SenseSim();
  ~SenseSim();
};

#endif

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
void SenseSim::odomSubCB(const nav_msgs::Odometry& odom_msg){
  // Calculate the time since the last encoder simulation. This is need since the new
  // encoder measurement is simulated using the velocity of the robot.
  double dtEnc = dtEncoder(odom_msg.header.stamp);

  // Create sensor messages to publish
  snowmower_msgs::EncMsg enc_msg;
  sensor_msgs::Imu imu_msg;

  // Simulate sensors!
  // Set Imu reading to velocity plus some random noise.
  imu_msg.header.stamp = odom_msg.header.stamp;
  imu_msg.header.frame_id = "base_christa";
  imu_msg.angular_velocity.z = odom_msg.twist.twist.angular.z;
  // Set encoder measurement based on previous encoder count and v and omega plus noise
  enc_msg.header.stamp = odom_msg.header.stamp;
  enc_msg.header.frame_id = "base_link";
  double v = odom_msg.twist.twist.linear.x;
  double omega = odom_msg.twist.twist.angular.z;
  enc_msg.right = enc_pre_.right + ((v - (b_/2)*omega) * dtEnc) * tpmRight_;
  ROS_INFO_STREAM("RIGHT: v = " << v << ", omega = " << omega <<", b = " << b_ << 
		  ", dtEnc = " << dtEnc << ", tpmRight = " << tpmRight_ << 
		  ", enc_msg.right = " <<enc_msg.right);
  enc_msg.left  = enc_pre_.left  + ((v + (b_/2)*omega) * dtEnc) * tpmLeft_;
  ROS_INFO_STREAM("LEFT:  v = " << v << ", omega = " << omega << ", b = " << b_ << 
		  ", dtEnc = " << dtEnc << ", tpmLeft  = " << tpmLeft_ << 
		  ", enc_msg.left  = " << enc_msg.left);
  // Store current encoder measurement as enc_pre_
  enc_pre_ = enc_msg;

  // Publish IMU and wheel encoder messages
  imuPub_.publish(imu_msg);
  encPub_.publish(enc_msg);
}

/* odomSubCB - Odom callback function *
 * When an nav_msgs::Odometry message arrives on the topic odom, simulate an
 * IMU and encoder measurement and publish them.
 */
void SenseSim::odomMapSubCB(const nav_msgs::Odometry& msg){
  // Simulate Decawave stuff
}

void SenseSim::init(){
  // Get parameters from launch file or set defaults
  if(!private_nh_.getParam("enc_std", enc_std_))
    enc_std_ = 1;
  if(!private_nh_.getParam("imu_std", imu_std_))
    imu_std_ = 1;
  if(!private_nh_.getParam("dw_std", imu_std_))
    imu_std_ = 1;
  if(!private_nh_.getParam("track_width", b_))
    b_ = 1;
  if(!private_nh_.getParam("tpm_right", tpmRight_))
    tpmRight_ = 1;
  if(!private_nh_.getParam("tpm_left", tpmLeft_))
    tpmLeft_ = 1;
  std::cout << "imu_std     = " << imu_std_ << std::endl;
  std::cout << "enc_std     = " << enc_std_ << std::endl;
  std::cout << "dw_std      = " << dw_std_ << std::endl;
  std::cout << "track_width = " << b_ << std::endl;
  std::cout << "tpm_right   = " << tpmRight_ << std::endl;
  std::cout << "tpm_left    = " << tpmLeft_ << std::endl;
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
  init();
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

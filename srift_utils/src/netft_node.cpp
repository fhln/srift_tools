/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/**
 * Simple stand-alone ROS node that takes data from NetFT sensor and
 * Publishes it ROS topic
 */

#include "ros/ros.h"
#include "srift_driver/srift_driver.h"
#include "srift_driver/srift_bias.h"
#include "geometry_msgs/WrenchStamped.h"
#include "diagnostic_msgs/DiagnosticArray.h"
#include "diagnostic_updater/DiagnosticStatusWrapper.h"
#include <unistd.h>
#include <iostream>
#include <memory>
#include <boost/program_options.hpp>

namespace po = boost::program_options;
using namespace std;


int main(int argc, char **argv)
{
  ros::init(argc, argv, "srift_node");

  ros::NodeHandle nh;
  std::string frame_id;
  std::string sensor_type;

  nh.param<std::string>(nh.getNamespace() + "/frame_id",frame_id,"");
  nh.param<std::string>(nh.getNamespace() + "/sensor_type",sensor_type,"");
  std::cout <<sensor_type<<std::endl;

  bool              b_bias_on_startup;
  float             pub_rate_hz;
  double            rot;
  double            alpha;
  tf::Vector3       scale_F;
  string            address;

  po::options_description desc("Options");
  desc.add_options()
  ("help", "display help")
  ("rate", po::value<float>(&pub_rate_hz)->default_value(500.0),
   "set publish rate (in hertz)")
  ("wrench", "publish older Wrench message type instead of WrenchStamped")
  ("bias",po::value<bool>(&b_bias_on_startup)->default_value(false),
   "if true computes the bias and substracts it at every time step from the signal")
  ("rot",po::value<double>(&rot)->default_value(0.0),
   " roation of the frame of reference of force torque vectors")
  ("alpha",po::value<double>(&alpha)->default_value(0.0),
   "alpha of exponential smoother, alpha in [0,1]" )
  ("scale_x",po::value<double>(&scale_F[0])->default_value(1.0),
   " x-axis scale factor [-1 or 1]")
  ("scale_y",po::value<double>(&scale_F[1])->default_value(1.0),
   " y-axis scale factor [-1 or 1]")
  ("scale_z",po::value<double>(&scale_F[2])->default_value(1.0),
   " z-axis scale factor [-1 or 1]")
  ("address", po::value<string>(&address), "IP address of  sri ft sensor")
  ;

  po::positional_options_description p;
  p.add("address",  1);

  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).options(desc).positional(p).run(),
            vm);
  po::notify(vm);

  if (vm.count("help")) {
    cout << desc << endl;
    //usage(progname);
    exit(EXIT_SUCCESS);
  }

  if (!vm.count("address")) {
    cout << desc << endl;
    cerr << "Please specify address of ft sensor device" << endl;
    exit(EXIT_FAILURE);
  }

  bool publish_wrench = false;
  if (vm.count("wrench")) {
    publish_wrench = true;
    ROS_WARN("Publishing FT sensor data as geometry_msgs::Wrench is deprecated");
  }

  std::auto_ptr<SRI_ft_Driver::SRIDriver> netft(new SRI_ft_Driver::SRIDriver(address,sensor_type));
  ros::Publisher pub;
  if (publish_wrench) {
    pub = nh.advertise<geometry_msgs::Wrench>("srift_data", 10);
  } else {
    pub = nh.advertise<geometry_msgs::WrenchStamped>("srift_data", 10);
  }
  ros::Rate pub_rate(pub_rate_hz);
  geometry_msgs::WrenchStamped data;

  ros::Duration                    diag_pub_duration(1.0);
  ros::Publisher                   diag_pub =
    nh.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 2);
  diagnostic_msgs::DiagnosticArray diag_array;
  diag_array.status.reserve(1);
  diagnostic_updater::DiagnosticStatusWrapper diag_status;
  ros::Time                                   last_diag_pub_time(
    ros::Time::now());

  /// Function to compute the BIAS in and subscract it.

  SRI_ft_Driver::NetFTRDTDriverBias        bias(nh,rot,scale_F,alpha);


  if(b_bias_on_startup) {
    bias.set_compute_bias(true);
  }
  while (ros::ok()) {
      netft->getData(data);
      // set the frame_id field (from rosparam loaded above)
      // defaults to empty string
      data.header.frame_id = frame_id;
      pub.publish(data);


     //  if (publish_wrench) {
    //     pub.publish(data.wrench);
    //     ROS_INFO("published!");
    //   } else {
    //     //bias.compute_bias(data.wrench);
    //     //bias.update(data.wrench);
    //     pub.publish(data);
    //     ROS_INFO("with bias published!");
    // }



    // ros::Time current_time(ros::Time::now());
    // if ( (current_time - last_diag_pub_time) > diag_pub_duration ) {
    //   diag_array.status.clear();
    //   netft->diagnostics(diag_status);
    //   diag_array.status.push_back(diag_status);
    //   diag_array.header.stamp = ros::Time::now();
    //   diag_pub.publish(diag_array);
    //   last_diag_pub_time = current_time;
    // }

    //ros::spinOnce();
       pub_rate.sleep();
  }

  return 0;
}

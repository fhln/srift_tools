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

#ifndef NETFT_RDT_DRIVER
#define NETFT_RDT_DRIVER

#include <boost/asio.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/condition.hpp>
#include <string>

#include "diagnostic_updater/DiagnosticStatusWrapper.h"
#include "geometry_msgs/WrenchStamped.h"
#include "geometry_msgs/Wrench.h"
#include "srift_driver/TCPClient.h"
#include <signal.h>
#include <Eigen/Dense>
using namespace Eigen;

namespace SRI_ft_Driver
{

class SRIDriver
{
public:
  // Start receiving data from SRI device
  SRIDriver(const std::string &address, const std::string &sensor_type);

  ~SRIDriver();

  //! Get newest realtime data from SRI device
  void getData(geometry_msgs::WrenchStamped &data);

  //! Add device diagnostics status wrapper, unfinished for SRI
  void diagnostics(diagnostic_updater::DiagnosticStatusWrapper &d);

  bool waitForNewData(void);

  geometry_msgs::WrenchStamped raw_FT_data;   //! Newest data received from  device
protected:
  void recvThreadFunc(void);


  enum {RDT_PORT=4008};
  std::string address_;
  std::string sensor_type_;
 
  boost::mutex mutex_;
  boost::thread recv_thread_;
 
  volatile bool stop_recv_thread_;
  //! True if recv loop is still running
  bool recv_thread_running_;
  //! Set if recv thread exited because of error
  std::string recv_thread_error_msg_; 

  //! Incremental counter for wrench header
  unsigned seq_counter_;


   TCPClient* tcp;
   void sig_exit(int s);
  bool ConfigSystem(void);
 

  //! Last time diagnostics was published
  ros::Time last_diag_pub_time_;
};


} 


#endif 

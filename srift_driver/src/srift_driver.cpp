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

#include "srift_driver/srift_driver.h"
#include <stdint.h>
#include <exception>
#include "srift_driver/M812x_def.h"
#include <time.h>

using boost::asio::ip::udp;

namespace SRI_ft_Driver
{

  bool SRIDriver::waitForNewData()
  {
    return true;
  }

   void SRIDriver::sig_exit(int s)
  {
    tcp->exit();
    exit(0);
  }

  bool SRIDriver::ConfigSystem(void)
  {
    bool debug = false;
     //string rec = tcp.receive();
  tcp->Send("AT+SGDM=(A01,A02,A03,A04,A05,A06);C;1;(WMA:1)\r\n");
  string rec = tcp->read();
  if( rec != "" )
  {
    if(debug)
      std::cout << "Server Response:" << rec << endl;
  } else
  {
    std::cout << "Server Not Response:"  << endl;
  }

  tcp->Send("AT+AMPZ=?\r\n");
  if(tcp->GetChParameter(m_dAmpZero))
  {
    if(debug)
      std::cout << "AT+AMPZ=: " << m_dAmpZero << endl;
  }else{
    std::cout << "Server no Response:" << endl;
  }

  tcp->Send("AT+CHNAPG=?\r\n");
  if(tcp->GetChParameter(m_dChnGain))
  {
    if(debug)
      std::cout << "AT+CHNAPG=: " << m_dChnGain << endl;
  }else{
    std::cout << "Server no Response:" << endl;
  }
  tcp->Send("AT+GOD\r\n");
  if(tcp->GetADCounts(m_nADCounts))
  {
    if(debug)
      std::cout << "AD value:" << m_nADCounts << endl;
  }else{
    std::cout << "Server no Response:" << endl;
  }

  tcp->Send("AT+SENS=?\r\n");
  if(tcp->GetChParameter(m_dSens))
    {
      if(debug)
        std::cout << "Sensivity value:" << m_dSens << endl;
    }else{
    std::cout << "Server no Response:" << endl;
  }

  tcp->Send("AT+EXMV=?\r\n");
  if(tcp->GetChParameter(m_dExc))
    {
      if(debug)
        std::cout << "AT+EXMV=: " << m_dExc << endl;
    }else{
    std::cout << "Server no Response:" << endl;
  }

  tcp->Send("AT+SMPR=500\r\n");
  if(tcp->GetSamplingRate())
    {
      //ROS_INFO("%s", samplerating_info_.c_str());
    }else{
    std::cout << "sampling rate set failed!"<<std::endl;
  }

  for (unsigned  int i = 0; i < 6; ++i) {
    m_dResultChValue(i) = 1000*( (m_nADCounts(i) - m_dAmpZero(i)) / (double)65535*(double)5 ) /(m_dExc(i))/ m_dChnGain(i);
    if(debug)
      printf("channel gain[%d]: %0.5f\n",i,m_dChnGain(i));
  }
  if(debug)
    std::cout << "result value is : "  <<m_dResultChValue << std::endl;

  //ROS_INFO("sensor type is %s", sensor_type_.c_str());
  if(this->sensor_type_=="100")  //M4313 ,TODO
    {
      m_dDecouplingCoefficient<<332.00220,-0.46851,-0.98522,3.60149,1.80495,-0.47651,
        0.68642,331.69588,-0.01472,-1.61450,4.59103,0.79022,
        4.00870,0.77786,228.46148,0.72871,90.22410,-2.42418,
        0.00605,-0.16531,0.11812,3.83920,0.01424,-0.08493,
        0.09159,-0.02124,-0.08898,0.01595,3.82423,-0.05152,
        -0.73979,-0.60347,-0.06308,-0.00402,0.00097,8.16786;
    }
  else if(this->sensor_type_=="40") //M3816, TODO
    {
      m_dDecouplingCoefficient<< 0.291821, -33.484164,-0.039229,30.957136, -0.125799,0.002550,
        0.072209,-19.330935,0.293554,-17.961256,0.278808, 36.184427,
        -53.516346,-0.210070,-54.313393,-0.204032,-52.571922,-0.316568,
        -2.131327,-0.004751,2.123981,0.023198, -0.004796,-0.016404,
        -1.221902,-0.018175,-1.240196,0.009643,2.408973,0.003018,
        -0.011847,1.745574,-0.007536,1.601929, 0.009660,1.623051;
    }
  else
    {
      ROS_ERROR("sensor_type is wrong!!!!!");
    }
  if(debug)
    std::cout << "Decoupling matrix is set as: " <<endl <<m_dDecouplingCoefficient << std::endl;
  m_dDecouplingValue=m_dResultChValue*m_dDecouplingCoefficient.transpose();

  if(debug)
    std::cout << "force is: " <<endl <<m_dDecouplingValue << std::endl;
  return true;
  }

  SRIDriver::SRIDriver(const std::string &address,const std::string &sensor_type) :
  address_(address),
  sensor_type_(sensor_type),
  stop_recv_thread_(false),
  recv_thread_running_(false),
  seq_counter_(0),
  last_diag_pub_time_(ros::Time::now())
 
{
  tcp = new TCPClient();
  //signal(SIGINT, sig_exit);
  const char* ip_temp=address_.c_str();
  if(tcp->setup(ip_temp,4008)==true)
    {
      cout<<"Force sensor has been connected!"<<endl;
    }else{
    cout<<"Force sensor connection failed!!!!!!!!!!!!!!!!!"<<endl;
  }
  //initialize the setting of the force sensor
  ConfigSystem();
  //get real time force sensor data
  tcp->Send("AT+GSD\r\n");

  // Start receive thread

  //recv_thread_ = boost::thread(&SRIDriver::recvThreadFunc, this);

}


SRIDriver::~SRIDriver()
{
  // TODO stop transmission,
  // stop thread
  stop_recv_thread_ = true;
  if (!recv_thread_.timed_join(boost::posix_time::time_duration(0,0,1,0))) {
    ROS_WARN("Interrupting recv thread");
    recv_thread_.interrupt();
    if (!recv_thread_.timed_join(boost::posix_time::time_duration(0,0,1,0))) {
      ROS_WARN("Failed second join to recv thread");
    }
  }
  tcp->exit();
  delete tcp;
}




void SRIDriver::recvThreadFunc()
{
  try {
    recv_thread_running_ = true;
   

    while (!stop_recv_thread_) {
      getData(raw_FT_data);
      ROS_INFO("count: %ld", seq_counter_);
    }
  } catch (std::exception &e) {
    recv_thread_running_ = false;
    {
      boost::unique_lock<boost::mutex> lock(mutex_);
      recv_thread_error_msg_ = e.what();
    }
  }
}



void SRIDriver::getData(geometry_msgs::WrenchStamped &data)
{
  {
    //struct timespec tpstart;
    // struct timespec tpend;

    boost::unique_lock<boost::mutex> lock(mutex_);

    //clock_gettime(CLOCK_MONOTONIC, &tpstart);
    tcp->readrecieveBuffer(m_nADCounts);
    //clock_gettime(CLOCK_MONOTONIC,&tpend);

    //double time_last = 1000.0*(tpend.tv_sec-tpstart.tv_sec)+(tpend.tv_nsec-tpstart.tv_nsec)/1000000.0;
    //ROS_INFO("%0.6f",time_last);

    for (unsigned  int i = 0; i < 6; ++i) {
      m_dResultChValue(i) = 1000*( (m_nADCounts(i) - m_dAmpZero(i)) / (double)65535*(double)5 )/(m_dExc(i)) / m_dChnGain(i);
    }
    m_dDecouplingValue=m_dResultChValue*m_dDecouplingCoefficient.transpose();
    //std::cout << "force is: " <<endl <<m_dDecouplingValue << std::endl;
    data.header.seq = seq_counter_++;
    data.header.stamp = ros::Time::now();
    data.wrench.force.x = double(m_dDecouplingValue(0,0));
    data.wrench.force.y = double(m_dDecouplingValue(0,1));
    data.wrench.force.z = double(m_dDecouplingValue(0,2));
    data.wrench.torque.x = double(m_dDecouplingValue(0,3));
    data.wrench.torque.y = double(m_dDecouplingValue(0,4));
    data.wrench.torque.z = double(m_dDecouplingValue(0,5));

    seq_counter_++;
    // ROS_INFO("count:%ld, x:%0.4f, y:%0.4f, z:%0.4f, tx:%0.4f, ty:%0.4f, tz:%0.4f", seq_counter_, data.wrench.force.x, data.wrench.force.y, data.wrench.force.z,data.wrench.torque.x, data.wrench.torque.y, data.wrench.torque.z);

  }
}


void SRIDriver::diagnostics(diagnostic_updater::DiagnosticStatusWrapper &d)
{
  // Publish diagnostics
  d.name = "SRI Driver : " + address_;

  d.summary(d.OK, "OK");
  //d.hardware_id = "0";

  if (!recv_thread_running_) {
    d.mergeSummaryf(d.ERROR, "Receive thread has stopped : %s",
                    recv_thread_error_msg_.c_str());
  }
  ros::Time current_time(ros::Time::now());

  d.clear();
  d.addf("IP Address", "%s", address_.c_str());

  geometry_msgs::WrenchStamped data;
  data = raw_FT_data;
  d.addf("Force X (N)",   "%f", data.wrench.force.x);
  d.addf("Force Y (N)",   "%f", data.wrench.force.y);
  d.addf("Force Z (N)",   "%f", data.wrench.force.z);
  d.addf("Torque X (Nm)", "%f", data.wrench.torque.x);
  d.addf("Torque Y (Nm)", "%f", data.wrench.torque.y);
  d.addf("Torque Z (Nm)", "%f", data.wrench.torque.z);
  last_diag_pub_time_ = current_time;
}


} // end namespace SRI_ft_driver


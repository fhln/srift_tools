
/*
Copyright (c) 2016, Los Alamos National Security, LLC
All rights reserved.
Copyright 2016. Los Alamos National Security, LLC. This software was produced under U.S. Government contract DE-AC52-06NA25396 for Los Alamos National Laboratory (LANL), which is operated by Los Alamos National Security, LLC for the U.S. Department of Energy. The U.S. Government has rights to use, reproduce, and distribute this software.  NEITHER THE GOVERNMENT NOR LOS ALAMOS NATIONAL SECURITY, LLC MAKES ANY WARRANTY, EXPRESS OR IMPLIED, OR ASSUMES ANY LIABILITY FOR THE USE OF THIS SOFTWARE.  If software is modified to produce derivative works, such modified software should be clearly marked, so as not to confuse it with the version available from LANL.

Additionally, redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer. 
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution. 
3. Neither the name of Los Alamos National Security, LLC, Los Alamos National Laboratory, LANL, the U.S. Government, nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission. 

THIS SOFTWARE IS PROVIDED BY LOS ALAMOS NATIONAL SECURITY, LLC AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL LOS ALAMOS NATIONAL SECURITY, LLC OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

Authors: Alex von Sternberg, Andy Zelenak
*/

#include "netft_utils.h"


int main(int argc, char **argv)
{
  // Initialize the ros netft_utils_node
  ros::init(argc, argv, "netft_utils_node");

  // Instantiate utils class
  ros::NodeHandle nh("~");
  NetftUtils utils(nh);
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Initialize utils
  utils.initialize();
  ROS_INFO("rawft topic data is without bias compensated or gravity compensated!!! just for test ");
  ROS_INFO("transform topic data should be used!");

  // Main ros loop
  ros::Rate loop_rate(500);
  //ros::Time l82.28626990867019ast;0x00007f495e719530 ***
  while(ros::ok())
  {
    utils.update();
    loop_rate.sleep();
    //ros::Time curr = ros::Time::now();
    //ROS_INFO_STREAM("Loop time: " <<  curr.toSec()-last.toSec());
    //last = curr;
  }

  return 0;
}

NetftUtils::NetftUtils(ros::NodeHandle nh) :
  n(nh),
  isFilterOn(false),
  deltaTFilter(0.0),
  cutoffFrequency(0.0),
  newFilter(false),
  isBiased(false),
  isGravityBiased(false),
  isNewBias(false),
  isNewGravityBias(false),
  cancel_count(MAX_CANCEL),
  cancel_wait(MAX_WAIT),
  forceMaxB(10.0),
  torqueMaxB(0.8),
  forceMaxU(50.0),
  torqueMaxU(5.0),
  payloadWeight(0.),
  payloadLeverArm(0.)
{
  nh.param("isFilterOn", isFilterOn, false);
  nh.param("deltaTFilter", deltaTFilter, 0.0);
  nh.param("cutoffFrequency", cutoffFrequency, 0.0);
  nh.param("newFilter", newFilter, false);
  nh.param("isBiased", isBiased, false);
  nh.param("isNewGravityBias", isNewGravityBias, false);
  nh.param("cancel_count", cancel_count, 5);
  nh.param("cancel_wait", cancel_wait, 100);
  nh.param("forceMaxB", forceMaxB, 250.0);
  nh.param("torqueMaxB", torqueMaxB, 14.0);
  nh.param("forceMaxU", forceMaxU, 260.0);
  nh.param("torqueMaxU", torqueMaxU, 16.0);
  nh.param("payloadWeight", payloadWeight, 0.0);
  nh.param("payloadLeverArm", payloadLeverArm, 0.0);
  nh.param("biasForceX", bias.wrench.force.x, 0.0);
  nh.param("biasForceY", bias.wrench.force.y, 0.0);
  nh.param("biasForceZ", bias.wrench.force.z, 0.0);
  nh.param("biasTorqueX", bias.wrench.torque.x, 0.0);
  nh.param("biasTorqueY", bias.wrench.torque.x, 0.0);
  nh.param("biasTorqueZ", bias.wrench.torque.x, 0.0);
  nh.param("ft_frame", ft_frame, std::string("ft_frame"));
  nh.param("world_frame", world_frame, std::string("base_link"));
  nh.param("ftAddress", ftAddress, std::string("192.168.0.102"));
  nh.param("ownFilterOn", ownFilterOn, true);
  nh.param("FilterSequenceCount", FilterSequenceCount, 5);
  //ROS_INFO("ft frame is %s", ft_frame.c_str());
  //ROS_INFO("cutoffFrequency %0.1f", cutoffFrequency);
  SensorIsActive=false;
}

NetftUtils::~NetftUtils()
{
   if(listener != NULL)
     delete listener;
   // if(lp != NULL)
   //   delete lp;
  delete srift_sensor;
}

bool NetftUtils::startFTsensor()
{
  try
    {
      //const std::string& ftAddress1=ftAddress;
      const std::string sensor_type("100");
      srift_sensor = new SRI_ft_Driver::SRIDriver(ftAddress,sensor_type);
      SensorIsActive = true;
    }
  catch(std::runtime_error)
    {
      ROS_ERROR_STREAM("Could not access data from netft_driver");
      return false;
    }
  return true;
}

void NetftUtils::initialize()
{
  if(isFilterOn)
    {
      lp = new LPFilter(deltaTFilter,cutoffFrequency,6);
      filter_ = new filters::MultiChannelMeanFilter<double>();

      int channels = 6;
      const std::string param_name("FifthOrder");
      filter_->MultiChannelFilterBase::configure(channels, param_name, n);
      for(int i=0;i<FilterSequenceCount;i++)
        {
          raw_data_collection.push_back(zero_wrench);
        }
      myfilter = new MyMeanFilter();
    }
  startFTsensor();

  //Zero out the zero wrench
  zero_wrench.wrench.force.x = 0.0;
  zero_wrench.wrench.force.y = 0.0;
  zero_wrench.wrench.force.z = 0.0;
  zero_wrench.wrench.torque.x = 0.0;
  zero_wrench.wrench.torque.y = 0.0;
  zero_wrench.wrench.torque.z = 0.0;

  //Initialize cancel message
  cancel_msg.toCancel = false;

  //Listen to the transfomation from the ft sensor to world frame.
  listener = new tf::TransformListener(ros::Duration(300));

  //Subscribe to the NetFT topic.
  //raw_data_sub = n.subscribe("/ft_sensor/srift_data",100, &NetftUtils::netftCallback, this);



  //Publish on the /netft_transformed_data topic. Queue up to 100000 data points
  netft_rawft_tool_data_pub  = n.advertise<geometry_msgs::WrenchStamped>("rawft_tool", 100000);
  netft_raw_tool_filted_data_pub = n.advertise<geometry_msgs::WrenchStamped>("raw_tool_filted", 100000);
  netft_raw_world_filted_data_pub = n.advertise<geometry_msgs::WrenchStamped>("raw_world_filted", 100000);
  netft_world_data_pub = n.advertise<geometry_msgs::WrenchStamped>("transformed_world", 100000);
  netft_tool_data_pub = n.advertise<geometry_msgs::WrenchStamped>("transformed_tool", 100000);
  netft_cancel_pub = n.advertise<srift_utils::Cancel>("cancel", 100000);

  //Advertise bias and threshold services
  bias_service = n.advertiseService("bias", &NetftUtils::fixedOrientationBias, this);
  bias_outside_service = n.advertiseService("input_ftbias", &NetftUtils::getBiasOutside, this); //从外部读取偏置信息

  gravity_comp_service = n.advertiseService("gravity_comp", &NetftUtils::compensateForGravity, this);
  set_max_service = n.advertiseService("set_max_values", &NetftUtils::setMax, this);
  theshold_service = n.advertiseService("set_threshold", &NetftUtils::setThreshold, this);
  weight_bias_service = n.advertiseService("set_weight_bias", &NetftUtils::setWeightBias, this);
  get_weight_service = n.advertiseService("get_weight", &NetftUtils::getWeight, this);
  filter_service = n.advertiseService("filter", &NetftUtils::setFilter, this);
}

void NetftUtils::setUserInput(std::string world, std::string ft, double force, double torque)
{
  world_frame = world;
  ft_frame = ft;
  if(force != 0.0)
  {
    forceMaxU = force;
  }
  if(torque != 0.0)
  {
    torqueMaxU = torque;
  }
}

void NetftUtils::update()
{
  netftCallback();
  // Check for a filter
  //ROS_INFO("new filter is %d", newFilter);
  if(newFilter)
  {
    delete lp; lp=NULL;
    lp = new LPFilter(deltaTFilter,cutoffFrequency,6);
    newFilter = false;
  }

  // Look up transform from ft to world frame
  tf::StampedTransform tempTransform;
  try
  {
    listener->waitForTransform(world_frame, ft_frame, ros::Time(0), ros::Duration(1.0));
    listener->lookupTransform(world_frame, ft_frame, ros::Time(0), tempTransform);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s",ex.what());
  }

  // Set translation to zero before updating value
  tempTransform.setOrigin(tf::Vector3(0.0,0.0,0.0));
  ft_to_world = tempTransform;

  checkMaxForce();

  // Publish transformed dat
  netft_rawft_tool_data_pub.publish(rawft_data_tool);
  netft_raw_tool_filted_data_pub.publish(raw_data_tool);
  netft_raw_world_filted_data_pub.publish( raw_data_world );
  netft_world_data_pub.publish( tf_data_world );
  netft_tool_data_pub.publish( tf_data_tool );
  netft_cancel_pub.publish( cancel_msg );
  //ROS_INFO("cutoffFrequency %0.1f", cutoffFrequency);
  ros::spinOnce();
}

void NetftUtils::copyWrench(geometry_msgs::WrenchStamped &in, geometry_msgs::WrenchStamped &out, geometry_msgs::WrenchStamped &bias)
{
  out.header.stamp = in.header.stamp;
  out.header.frame_id = in.header.frame_id;
  out.wrench.force.x = in.wrench.force.x - bias.wrench.force.x;
  out.wrench.force.y = in.wrench.force.y - bias.wrench.force.y;
  out.wrench.force.z = in.wrench.force.z - bias.wrench.force.z;
  out.wrench.torque.x = in.wrench.torque.x - bias.wrench.torque.x;
  out.wrench.torque.y = in.wrench.torque.y - bias.wrench.torque.y;
  out.wrench.torque.z = in.wrench.torque.z - bias.wrench.torque.z;
}

void NetftUtils::applyThreshold(double &value, double thresh)
{
  if(value <= thresh && value >= -thresh)
  {
    value = 0.0;
  }
}

void NetftUtils::transformFrame(geometry_msgs::WrenchStamped in_data, geometry_msgs::WrenchStamped &out_data, char target_frame)
{
  tf::Vector3 tempF;
  tf::Vector3 tempT;
  tempF.setX(in_data.wrench.force.x);
  tempF.setY(in_data.wrench.force.y);
  tempF.setZ(in_data.wrench.force.z);
  tempT.setX(in_data.wrench.torque.x);
  tempT.setY(in_data.wrench.torque.y);
  tempT.setZ(in_data.wrench.torque.z);
  if(target_frame == 'w')
  {
      out_data.header.frame_id = world_frame;
      tempF = ft_to_world * tempF;
      tempT = ft_to_world * tempT;
  }
  else if(target_frame == 't')
  {
      out_data.header.frame_id = ft_frame;
      tempF = ft_to_world.inverse() * tempF;
      tempT = ft_to_world.inverse() * tempT;
  }	
  out_data.header.stamp = in_data.header.stamp;
  out_data.wrench.force.x = tempF.getX();
  out_data.wrench.force.y = tempF.getY();
  out_data.wrench.force.z = tempF.getZ();
  out_data.wrench.torque.x = tempT.getX();
  out_data.wrench.torque.y = tempT.getY();
  out_data.wrench.torque.z = tempT.getZ();
}

// Runs when a new datapoint comes in
void NetftUtils::netftCallback()
{
  // Filter data
  std::vector<double> tempData;
  geometry_msgs::WrenchStamped data;
  tempData.resize(6);
  srift_sensor->getData(data);

  rawft_data_tool = data;

  tempData.at(0) = data.wrench.force.x;
  tempData.at(1) = data.wrench.force.y;
  tempData.at(2) = data.wrench.force.z;
  tempData.at(3) = data.wrench.torque.x;
  tempData.at(4) = data.wrench.torque.y;
  tempData.at(5) = data.wrench.torque.z;

  if(!ownFilterOn)
    {
      if(isFilterOn && !newFilter)
        lp->update(tempData,tempData);
      // Copy tool frame data. apply negative to x data to follow right hand rule convention (ft raw data does not)
      raw_data_tool.header.stamp = data.header.stamp;
      raw_data_tool.header.frame_id = ft_frame;
      raw_data_tool.wrench.force.x = tempData.at(0);
      raw_data_tool.wrench.force.y = tempData.at(1);
      raw_data_tool.wrench.force.z = tempData.at(2);
      raw_data_tool.wrench.torque.x = tempData.at(3);
      raw_data_tool.wrench.torque.y = tempData.at(4);
      raw_data_tool.wrench.torque.z = tempData.at(5);
    }
  else
    {
      raw_data_collection.erase(raw_data_collection.begin());
      raw_data_collection.push_back(data);
      myfilter->getFiltedData(raw_data_collection,raw_data_tool);
      raw_data_tool.header.stamp = data.header.stamp;
      raw_data_tool.header.frame_id = ft_frame;
    }



  
  // Copy in new netft data in tool frame and transform to world frame
  transformFrame(raw_data_tool, raw_data_world, 'w');


  if (isGravityBiased) // Compensate for gravity. Assumes world Z-axis is up TODO: 加上自己的重力补偿代码和配置,可以参考force_torque_tools 包
  {
      // Gravity moment = (payloadLeverArm) cross (payload force)  <== all in the sensor frame. Need to convert to world later
      // Since it's assumed that the CoM of the payload is on the sensor's central axis, this calculation is simplified.
      double gravMomentX = -payloadLeverArm*tf_data_tool.wrench.force.y;
      double gravMomentY = payloadLeverArm*tf_data_tool.wrench.force.x;
    
      // Subtract the gravity torques from the previously-calculated wrench in the tool frame
      tf_data_tool.wrench.torque.x = tf_data_tool.wrench.torque.x - gravMomentX;
      tf_data_tool.wrench.torque.y = tf_data_tool.wrench.torque.y - gravMomentY;
    
      // Convert to world to account for the gravity force. Assumes world-Z is up.
      //ROS_INFO_STREAM("gravity force in the world Z axis: "<< payloadWeight);
      transformFrame(tf_data_tool, tf_data_world, 'w');
      tf_data_world.wrench.force.z = tf_data_world.wrench.force.z - payloadWeight;
    
      // tf_data_world now accounts for gravity completely. Convert back to the tool frame to make that data available, too
      transformFrame(tf_data_world, tf_data_tool, 't');
  }
  
  if (isBiased) // Apply the bias for a static sensor frame
  {
    // Get tool bias in world frame
    geometry_msgs::WrenchStamped world_bias;
    transformFrame(bias, world_bias, 'w');
    // Add bias and apply threshold to get transformed data
    copyWrench(raw_data_world, tf_data_world, world_bias);
    copyWrench(raw_data_tool, tf_data_tool, bias);
  }
  else // Just pass the data straight through
  {
    copyWrench(raw_data_world, tf_data_world, zero_wrench);
    copyWrench(raw_data_tool, tf_data_tool, zero_wrench);
  }
                  
  // Apply thresholds
  applyThreshold(tf_data_world.wrench.force.x, threshold.wrench.force.x);
  applyThreshold(tf_data_world.wrench.force.y, threshold.wrench.force.y);
  applyThreshold(tf_data_world.wrench.force.z, threshold.wrench.force.z);
  applyThreshold(tf_data_world.wrench.torque.x, threshold.wrench.torque.x);
  applyThreshold(tf_data_world.wrench.torque.y, threshold.wrench.torque.y);
  applyThreshold(tf_data_world.wrench.torque.z, threshold.wrench.torque.z);
  applyThreshold(tf_data_tool.wrench.force.x, threshold.wrench.force.x);
  applyThreshold(tf_data_tool.wrench.force.y, threshold.wrench.force.y);
  applyThreshold(tf_data_tool.wrench.force.z, threshold.wrench.force.z);
  applyThreshold(tf_data_tool.wrench.torque.x, threshold.wrench.torque.x);
  applyThreshold(tf_data_tool.wrench.torque.y, threshold.wrench.torque.y);
  applyThreshold(tf_data_tool.wrench.torque.z, threshold.wrench.torque.z);
  //ROS_INFO_STREAM("Callback time: " << tf_data_tool.header.stamp.toSec()-ros::Time::now().toSec());
}                 

bool NetftUtils::getBiasOutside(srift_utils::SetBiasOutside::Request &req, srift_utils::SetBiasOutside::Response &res)
{
  copyWrench(req.bias_outside, bias, zero_wrench);
  isBiased = true;

}

// Set the readings from the sensor to zero at this instant and continue to apply the bias on future readings.
// This doesn't account for gravity.
// Useful when the sensor's orientation won't change.
// Run this method when the sensor is stationary to avoid inertial effects.
// Cannot bias the sensor if gravity compensation has already been applied.
bool NetftUtils::fixedOrientationBias(srift_utils::SetBias::Request &req, srift_utils::SetBias::Response &res)
{                 
  if(req.toBias)  
  {           
    copyWrench(raw_data_tool, bias, zero_wrench); // Store the current wrench readings in the 'bias' variable, to be applied hereafter
    if(req.forceMax >= 0.0001) // if forceMax was specified and > 0
      forceMaxB = req.forceMax;
    if(req.torqueMax >= 0.0001)
      torqueMaxB = req.torqueMax; // if torqueMax was specified and > 0
    
    isNewBias = true;
    isBiased = true;
  }               
  else            
  {               
    copyWrench(zero_wrench, bias, zero_wrench); // Clear the stored bias if the argument was false
  }               

  res.success = true;
                  
  return true;    
}

// Calculate the payload's mass and center of mass so gravity can be compensated for, even as the sensor changes orientation.
// It's assumed that the payload's center of mass is located on the sensor's central access.
// Run this method when the sensor is stationary to avoid inertial effects.
// Cannot do gravity compensation if sensor has already been biased.
bool NetftUtils::compensateForGravity(srift_utils::SetBias::Request &req, srift_utils::SetBias::Response &res)
{

  if(req.toBias)
  {  
    if (isBiased)
    {
      ROS_ERROR("Cannot compensate for gravity if the sensor has already been biased, i.e. useful data was wiped out");
      res.success = false;
      return false;  
    }
    else  // Cannot compensate for gravity if the sensor has already been biased, i.e. useful data was wiped out 
    {
      // Get the weight of the payload. Assumes the world Z axis is up.
      payloadWeight = raw_data_world.wrench.force.z;
  
      // Calculate the z-coordinate of the payload's center of mass, in the sensor frame.
      // It's assumed that the x- and y-coordinates are zero.
      // This is a lever arm.
      payloadLeverArm = raw_data_tool.wrench.torque.y/raw_data_tool.wrench.force.x;
    
      isNewGravityBias = true;
      isGravityBiased = true;
    }
  }
  
  res.success = true;     
  return true;
}  

bool NetftUtils::setFilter(srift_utils::SetFilter::Request &req, srift_utils::SetFilter::Response &res)
{                 
  if(req.toFilter)  
  {
    newFilter = true;
    isFilterOn = true;
    deltaTFilter = req.deltaT;
    cutoffFrequency = req.cutoffFrequency;
  }               
  else            
  {               
    isFilterOn = false;
  }               
  
  return true;    
}  

bool NetftUtils::setMax(srift_utils::SetMax::Request &req, srift_utils::SetMax::Response &res)
{                 
  if(req.forceMax >= 0.0001)
    forceMaxU = req.forceMax;
  if(req.torqueMax >= 0.0001)
    torqueMaxU = req.torqueMax;
  
  res.success = true;             
  return true;    
}

bool NetftUtils::setWeightBias(srift_utils::SetBias::Request &req, srift_utils::SetBias::Response &res)
{                 
  if(req.toBias)  
  {               
    copyWrench(raw_data_tool, weight_bias, zero_wrench);
    copyWrench(raw_data_tool, bias, zero_wrench);
  }               
  else            
  {               
    copyWrench(zero_wrench, weight_bias, zero_wrench);
  }               
  res.success = true;
                  
  return true;    
}  
    
bool NetftUtils::getWeight(srift_utils::GetDouble::Request &req, srift_utils::GetDouble::Response &res)
{                 
  geometry_msgs::WrenchStamped carried_weight;
  copyWrench(raw_data_tool, carried_weight, weight_bias);
  res.weight = pow((pow(carried_weight.wrench.force.x, 2.0) + pow(carried_weight.wrench.force.y, 2.0) + pow(carried_weight.wrench.force.z, 2.0)), 0.5)/9.81*1000;
                  
  return true;    
}      
            
bool NetftUtils::setThreshold(srift_utils::SetThreshold::Request &req, srift_utils::SetThreshold::Response &res)
{                 
  threshold.wrench.force.x = req.data.wrench.force.x;
  threshold.wrench.force.y = req.data.wrench.force.y;
  threshold.wrench.force.z = req.data.wrench.force.z;
  threshold.wrench.torque.x = req.data.wrench.torque.x;
  threshold.wrench.torque.y = req.data.wrench.torque.y;
  threshold.wrench.torque.z = req.data.wrench.torque.z;
                  
  res.success = true;
                  
  return true;    
}

void NetftUtils::checkMaxForce()
{
  double fMag = pow((pow(tf_data_tool.wrench.force.x, 2.0) + pow(tf_data_tool.wrench.force.y, 2.0) + pow(tf_data_tool.wrench.force.z, 2.0)), 0.5);
  double tMag = pow((pow(tf_data_tool.wrench.torque.x, 2.0) + pow(tf_data_tool.wrench.torque.y, 2.0) + pow(tf_data_tool.wrench.torque.z, 2.0)), 0.5);
  double fMax;
  double tMax;
  if(isBiased && !isNewBias)
  {
    fMax = forceMaxB;
    tMax = torqueMaxB;
  }
  else
  {
    if(isBiased && isNewBias)
    {
      isNewBias = false;
    }

    fMax = forceMaxU; //50.0;
    tMax = torqueMaxU; //5.0;
  }

  // If max FT exceeded, send cancel unless we have just sent it MAX_CANCEL times
  //ROS_INFO("FMAG: %f TMAG: %f", fMag, tMag);
  if((fabs(fMag) > fMax || fabs(tMag) > tMax) && cancel_count > 0)
  {
    cancel_msg.toCancel = true;
    //ROS_INFO("Force torque violation. Canceling move.");
    ROS_INFO("FMAG: %f FMAX: %f TMAG:%f TMAX: %f count: %d wait: %d", fMag, fMax, tMag, tMax, cancel_count, cancel_wait);
    cancel_count-=1;
  }
  // If we have sent cancel MAX_CANCEL times, don't send cancel again for MAX_WAIT cycles
  else if(cancel_count == 0 && cancel_wait > 0 && cancel_wait <= MAX_WAIT)
  {
    cancel_msg.toCancel = false;
    cancel_wait-=1;
  }
  // If we have just finished waiting MAX_WAIT times, or the max force is no longer exceeded, reset cancel_count and cancel_wait
  else if(cancel_wait == 0 || !(fabs(fMag) > fMax || fabs(tMag) > tMax))
  {
    cancel_msg.toCancel = false;
    cancel_count = MAX_CANCEL;
    cancel_wait = MAX_WAIT;
  }
}               
                  

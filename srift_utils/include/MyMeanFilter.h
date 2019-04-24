#ifndef MYMEANFILTER_H
#define MYMEANFILTER
#include <vector>
#include <ros/ros.h>
#include <math.h>
#include "geometry_msgs/WrenchStamped.h"

class MyMeanFilter
{
public:
  MyMeanFilter(){}
  void getFiltedData(const std::vector<geometry_msgs::WrenchStamped>& data_collection, geometry_msgs::WrenchStamped& result);
};


#endif

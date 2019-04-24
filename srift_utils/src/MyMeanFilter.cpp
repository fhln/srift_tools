#include "MyMeanFilter.h"

void MyMeanFilter::getFiltedData(const std::vector<geometry_msgs::WrenchStamped>& data_collection, geometry_msgs::WrenchStamped& result)
{
  int length = data_collection.size();
  if(length <=0)
    {
      ROS_ERROR("filter sequence length is zero");
      return;
    }
  double sumFX=0.0;
  double sumFY=0.0;
  double sumFZ=0.0;
  double sumTX=0.0;
  double sumTY=0.0;
  double sumTZ=0.0;
  for(int i=0; i<length; i++)
  {
    const geometry_msgs::WrenchStamped& tempData = data_collection.at(i);
    sumFX+= tempData.wrench.force.x;
    sumFY+=tempData.wrench.force.y;
    sumFZ+=tempData.wrench.force.z;
    sumTX+=tempData.wrench.torque.x;
    sumTY+=tempData.wrench.torque.y;
    sumTZ+=tempData.wrench.torque.z;
  }
  result.wrench.force.x =sumFX/length;
  result.wrench.force.y = sumFY/length;
  result.wrench.force.z = sumFZ/length;
  result.wrench.torque.x = sumTX/length;
  result.wrench.torque.y = sumTY/length;
  result.wrench.torque.z = sumTZ/length;
}

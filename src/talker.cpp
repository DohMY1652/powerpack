#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <vector>

#include "Powerpack.h"


int main(int argc, char **argv)
{

  ros::init(argc, argv, "talker");
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<std_msgs::Int32MultiArray>("mpc_pwm", 10);
  ros::Rate loop_rate(100);

  while (ros::ok())
  {

    std::vector<int> data = {0,1,2,3,4};
    std_msgs::Int32MultiArray msg;
    msg.data.resize(data.size());
    for (size_t i = 0; i < data.size(); ++i) {
    msg.data[i] = data[i];
    }
    
    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}
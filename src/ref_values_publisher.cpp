#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "ref_values_publisher");
    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<std_msgs::Float32MultiArray>("/ref_values", 1);
    ros::Rate loop_rate(10); // 10 Hz

    while (ros::ok()) {
        std_msgs::Float32MultiArray msg;
        msg.data = {301.325,301.325, 301.325, 51.325,51.325, 51.325};
        pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

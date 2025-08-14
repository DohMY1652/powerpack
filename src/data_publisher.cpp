#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/UInt16MultiArray.h>

int main(int argc, char** argv)
{
    // ROS 초기화
    ros::init(argc, argv, "multi_topic_publisher_node");
    ros::NodeHandle nh;

    ros::Publisher sen_values_pub = nh.advertise<std_msgs::Float32MultiArray>("/sen_raw_values", 1);

    ros::Publisher ref_values_pub = nh.advertise<std_msgs::Float32MultiArray>("/mpc_ref_values", 1);

    ros::Publisher rl_values_pub = nh.advertise<std_msgs::UInt16MultiArray>("/rl_pwm", 1);


    // 주기적으로 데이터를 publish하는 루프 설정 (10Hz)
    ros::Rate rate(1000);  // 10Hz

    while (ros::ok())
    {
        std_msgs::Float32MultiArray sen_values_msg;
        sen_values_msg.data = {1.0, 1.0, 4.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};

        std_msgs::Float32MultiArray ref_values_msg;
        // ref_values_msg.data = {201.325, 201.325, 201.325, 201.325, 71.325, 71.325, 71.325, 71.325};
        ref_values_msg.data = {101.325, 151.325, 101.325, 101.325, 51.325, 101.325, 101.325, 101.325};

        std_msgs::UInt16MultiArray rl_pwm_msg;
        rl_pwm_msg.data = {10, 10};

        // /sen_values와 /ref_values에 각각 메시지 publish
        // sen_values_pub.publish(sen_values_msg);
        ref_values_pub.publish(ref_values_msg);
        // rl_values_pub.publish(rl_pwm_msg);

        // ROS_INFO("Data Publshed");
        // 주기 설정에 맞춰 sleep
        rate.sleep();
    }

    return 0;
}

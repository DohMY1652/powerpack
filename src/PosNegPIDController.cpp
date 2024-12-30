#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/UInt16MultiArray.h"

class PosNegPIDController {
public:
    PosNegPIDController(double kp, double ki, double kd)
        : kp_(kp), ki_(ki), kd_(kd), prev_error_(0.0), integral_(0.0) {}

    double compute(double setpoint, double measurement, double dt) {
        double error = setpoint - measurement;
        integral_ += error * dt;
        double derivative = (error - prev_error_) / dt;
        prev_error_ = error;
        double output = kp_ * error + ki_ * integral_ + kd_ * derivative;

        // 출력값을 0에서 100 사이로 제한
        if (output < 0.0) output = 0.0;
        if (output > 100.0) output = 100.0;

        return output;
    }

private:
    double kp_, ki_, kd_;
    double prev_error_;
    double integral_;
};

class ControllerNode {
public:
    ControllerNode(ros::NodeHandle& nh, double kp1, double ki1, double kd1, double kp2, double ki2, double kd2) {
        pid1_ = new PosNegPIDController(kp1, ki1, kd1);
        pid2_ = new PosNegPIDController(kp2, ki2, kd2);
        sub_ = nh.subscribe("sen_values", 100, &ControllerNode::sensorCallback, this);
        pub_ = nh.advertise<std_msgs::UInt16MultiArray>("rl_pwm", 100);
        last_time_ = ros::Time::now();
    }

    ~ControllerNode() {
        delete pid1_;
        delete pid2_;
    }

    double input_mapping(double input, double P_in, double P_out) {
        double delP = P_in - P_out;
        double u_min = 98.85 - 0.03191 * delP;
        double Q_max = -407.1 + 0.1922 * delP + 4.072 * 100;
        if (input >= Q_max) {
            return 1000;
        }
        else {
            return 10 * (input / Q_max * (100-u_min) + u_min);
        }
    }

    void sensorCallback(const std_msgs::Float32MultiArray::ConstPtr& msg) {
        if (msg->data.size() < 9) {
            ROS_WARN("Expected sen_values size at least 9, but got size %zu", msg->data.size());
            return;
        }

        ROS_INFO("Received sensor values: %f, %f",((msg->data[0]-pos_offset)*pos_gain+atm_offset),((msg->data[1]-neg_offset)*neg_gain+atm_offset));

        // 시간 계산
        ros::Time now = ros::Time::now();
        double dt = (now - last_time_).toSec();
        last_time_ = now;

        // 앞의 2개의 값으로 PID 계산
        double pwm1 = pid1_->compute(401.325, ((msg->data[0]-pos_offset)*pos_gain+atm_offset), dt);  // 첫 번째 PID
        double pwm2 = pid2_->compute(-1*(21.325-101.325), -1 * (((msg->data[1]-neg_offset)*neg_gain+atm_offset)- 101.325), dt);  // 두 번째 PID
        ROS_INFO("PID calculated");
        pwm1 = input_mapping(100 - pwm1, ((msg->data[0]-pos_offset)*pos_gain+atm_offset), 101.325) - 20;
        pwm2 = input_mapping(100 - pwm2, 101.325, ((msg->data[1]-neg_offset)*neg_gain+atm_offset)) - 20;
        // UInt16MultiArray로 결과를 publish
        std_msgs::UInt16MultiArray pwm_msg;
        pwm_msg.data.push_back(static_cast<uint16_t>(pwm1));
        pwm_msg.data.push_back(static_cast<uint16_t>(pwm2));

        pub_.publish(pwm_msg);
        ROS_INFO("pwm published");
    }

private:
    ros::Subscriber sub_;
    ros::Publisher pub_;
    PosNegPIDController* pid1_;
    PosNegPIDController* pid2_;
    ros::Time last_time_;
    double pos_offset = 1;
    double pos_gain = 250;
    double neg_offset = 1;
    double neg_gain = -25.25;
    double atm_offset = 101.325;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "pid_controller_node");
    ros::NodeHandle nh;

    // PID 파라미터를 직접 선언
    double kp1 = 1.0, ki1 = 0.1, kd1 = 0.01;
    double kp2 = 3.0, ki2 = 0.3, kd2 = 0.03;

    // ControllerNode 생성
    ControllerNode controller(nh, kp1, ki1, kd1, kp2, ki2, kd2);

    ros::spin();
    return 0;
}

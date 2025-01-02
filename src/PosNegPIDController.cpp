#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/UInt16MultiArray.h"

#include <yaml-cpp/yaml.h>
#include "DatabaseConfig.h"


class PIDController {
public:
    PIDController(double kp, double ki, double kd)
        : kp_(kp), ki_(ki), kd_(kd), prev_error_(0.0), integral_(0.0) {}

    double compute(double setpoint, double measurement, double dt) {
        double error = setpoint - measurement;
        integral_ += error * dt;
        double derivative = (error - prev_error_) / dt;
        prev_error_ = error;
        double output = kp_ * error + ki_ * integral_ + kd_ * derivative;

        // 출력값을 0에서 100 사이로 제한
        if (output < -50.0) output = -50.0;
        if (output > 50.0) output = 50.0;
        return output;
    }

private:
    double kp_, ki_, kd_;
    double prev_error_;
    double integral_;
};

class ControllerNode {
public:
    ControllerNode(ros::NodeHandle& nh, std::shared_ptr<DatabaseConfig> &databaseconfig) : atm_offset(101.325) {
        sub_ = nh.subscribe("sen_raw_values", 1, &ControllerNode::sensorCallback, this);
        pub_ = nh.advertise<std_msgs::UInt16MultiArray>("rl_pwm", 100);
        raw_pub_ = nh.advertise<std_msgs::UInt16MultiArray>("raw_rl_pwm", 100);
        last_time_ = ros::Time::now();

        std::vector<bool> system_parameters = databaseconfig->get_system_parameters();
        operating = system_parameters[2];

        std::vector<double> pid_gains = databaseconfig->get_pid_gains();
        pos_pid_ = new PIDController(pid_gains[1], pid_gains[2], pid_gains[3]);
        neg_pid_ = new PIDController(pid_gains[4], pid_gains[5], pid_gains[6]);

        pos_ref = pid_gains[7];
        neg_ref = pid_gains[8];

        std::vector<double> parameters = databaseconfig->get_sensor_parameters();
        pos_offset = parameters[1];
        pos_gain = parameters[2];
        neg_offset = parameters[3];
        neg_gain = parameters[4];


    }

    ~ControllerNode() {
        delete pos_pid_;
        delete neg_pid_;
    }

    double input_mapping(double input, double P_in, double P_out) {
        double delP = P_in - P_out;
        double u_min = 98.85 - 0.03191 * delP;
        double Q_max = -407.1 + 0.1922 * delP + 4.072 * 100;
        if (input >= Q_max) {
            return 10 * (Q_max / Q_max * (100-u_min) + u_min);
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
        // ROS_INFO("Received sensor raw values: %f, %f",((msg->data[0])),((msg->data[1])));
        // ROS_INFO("Received sensor values: %f, %f",((msg->data[0]-pos_offset)*pos_gain+atm_offset),((msg->data[1]-neg_offset)*neg_gain+atm_offset));

        // 시간 계산
        ros::Time now = ros::Time::now();
        double dt = (now - last_time_).toSec();
        last_time_ = now;

        // 앞의 2개의 값으로 PID 계산
        double pos_pwm = pos_pid_->compute(pos_ref, ((msg->data[0] - pos_offset) * pos_gain + atm_offset), dt);  // 첫 번째 PID
        double neg_pwm = -1* (neg_pid_->compute(neg_ref, ((msg->data[1] - neg_offset) * neg_gain + atm_offset), dt));  // 두 번째 PID
        ROS_INFO("reference values: %f, %f",((pos_ref)),((neg_ref)));
        ROS_INFO("Received sensor values: %f, %f",((msg->data[0]-pos_offset)*pos_gain+atm_offset),((msg->data[1]-neg_offset)*neg_gain+atm_offset));
        ROS_INFO("Pos_raw_pwm : %f", pos_pwm);
        ROS_INFO("Neg_raw_pwm : %f", neg_pwm);
        // pos_pwm = input_mapping(50 - pos_pwm, ((msg->data[0]-pos_offset)*pos_gain+atm_offset), atm_offset);
        // neg_pwm = input_mapping(50 - neg_pwm, atm_offset, ((msg->data[1]-neg_offset)*neg_gain+atm_offset));
        pos_pwm = 50 - pos_pwm;
        neg_pwm = 50 - neg_pwm;
        // UInt16MultiArray로 결과를 publish
        std_msgs::UInt16MultiArray pwm_msg;
        std_msgs::UInt16MultiArray raw_pwm_msg;
        raw_pwm_msg.data.push_back(static_cast<uint16_t>(pos_pwm));
        raw_pwm_msg.data.push_back(static_cast<uint16_t>(neg_pwm));
        if (operating) {
            pwm_msg.data.push_back(static_cast<uint16_t>(pos_pwm));
            pwm_msg.data.push_back(static_cast<uint16_t>(neg_pwm));
        } else {
            pwm_msg.data.push_back(static_cast<uint16_t>(0));
            pwm_msg.data.push_back(static_cast<uint16_t>(0));
        }
        pub_.publish(pwm_msg);
        raw_pub_.publish(raw_pwm_msg);
        ROS_INFO("Pos_pwm : %f", pos_pwm);
        ROS_INFO("Neg_pwm : %f", neg_pwm);
    }

private:
    ros::Subscriber sub_;
    ros::Publisher pub_;
    ros::Publisher raw_pub_;
    PIDController* pos_pid_;
    PIDController* neg_pid_;
    ros::Time last_time_;
    double pos_offset;
    double pos_gain;
    double neg_offset;
    double neg_gain;
    double atm_offset;

    double pos_ref;
    double neg_ref;

    bool operating = false;
};

int main(int argc, char** argv) {
    
    ros::init(argc, argv, "pid_controller_node");
    ros::NodeHandle nh;

    std::string yaml_file;
      if (!nh.getParam("yaml_file", yaml_file)) {
          ROS_ERROR("Could not find parameter 'yaml_file'");
          return 1;
      }

      YAML::Node config = YAML::LoadFile(yaml_file);
      std::shared_ptr<DatabaseConfig> databaseconfig = std::make_shared<DatabaseConfig>(config);

    // PID 파라미터를 직접 선언

    // ControllerNode 생성
    ControllerNode controller(nh, databaseconfig);

    ros::spin();
    return 0;
}

#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/UInt16MultiArray.h"

#include <yaml-cpp/yaml.h>
#include "DatabaseConfig.h"

class PIDController {
public:
    PIDController(double kp, double ki, double kd)
        : kp_(kp), ki_(ki), kd_(kd), prev_error_(0.0), integral_(0.0), output_min_(-50.0), output_max_(50.0) {}

    double compute(double setpoint, double measurement, double dt) {
        double error = setpoint - measurement;
        double derivative = (error - prev_error_) / dt;
        double output = kp_ * error + ki_ * integral_ + kd_ * derivative;

        // Anti-windup logic
        if (output > output_max_) {
            output = output_max_;
        } else if (output < output_min_) {
            output = output_min_;
        } else {
            integral_ += error * dt; // Accumulate integral only when output is not saturated
        }

        prev_error_ = error;
        ROS_INFO("Integral : %f", integral_);
        return output;
    }

    void setOutputLimits(double min, double max) {
        output_min_ = min;
        output_max_ = max;
    }

private:
    double kp_, ki_, kd_;
    double prev_error_;
    double integral_;
    double output_min_, output_max_;
};

class ControllerNode {
public:
    ControllerNode(ros::NodeHandle& nh, std::shared_ptr<DatabaseConfig>& databaseconfig)
        : atm_offset(101.325) {
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

        pos_pid_->setOutputLimits(-50.0, 50.0);
        neg_pid_->setOutputLimits(-50.0, 50.0);

        publisher = nh.advertise<std_msgs::Float32MultiArray>("rl_ref_values", 1);
        
        rl_ref  = databaseconfig->get_pid_gains();

        
    }

    ~ControllerNode() {
        delete pos_pid_;
        delete neg_pid_;
    }

    void sensorCallback(const std_msgs::Float32MultiArray::ConstPtr& msg) {
        std_msgs::Float32MultiArray rl_ref_data;
        rl_ref_data.data.resize(2);
        rl_ref_data.data[0] = static_cast<float_t> (rl_ref[7]);
        rl_ref_data.data[1] = static_cast<float_t> (rl_ref[8]);
        publisher.publish(rl_ref_data);
        if (msg->data.size() < 9) {
            ROS_WARN("Expected sen_values size at least 9, but got size %zu", msg->data.size());
            return;
        }

        ros::Time now = ros::Time::now();
        double dt = (now - last_time_).toSec();
        last_time_ = now;

        double pos_pwm = pos_pid_->compute(pos_ref, ((msg->data[0] - pos_offset) * pos_gain + atm_offset), dt);
        double neg_pwm = -1 * neg_pid_->compute(neg_ref, ((msg->data[1] - neg_offset) * neg_gain + atm_offset), dt);
        ROS_INFO("Pos_raw_pwm : %f", pos_pwm);
        ROS_INFO("Neg_raw_pwm : %f", neg_pwm);
        pos_pwm = 10 * (0.6 * 100 + 0.4 * (50 - pos_pwm));
        neg_pwm = 10 * (0.7 * 100 + 0.3 * (50 - neg_pwm));
        ROS_INFO("Pos_pwm : %f", pos_pwm);
        ROS_INFO("Neg_pwm : %f", neg_pwm);
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

    std::vector<double> rl_ref;

    ros::Publisher publisher;

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

    ControllerNode controller(nh, databaseconfig);

    ros::spin();
    return 0;
}

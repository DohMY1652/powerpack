#ifndef POWERPACK_H
#define POWERPACK_H

#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <memory>


#include "Sensor.h"
#include "PWM.h"
#include "ControlModule.h"
#include "ReferenceGoverner.h"
#include "DatabaseConfig.h"

class Powerpack {
public:
    Powerpack(ros::NodeHandle& nh, std::shared_ptr<DatabaseConfig> &databaseconfig);
    ~Powerpack();

    std::vector<double> get_sensor_data() const;
    std::vector<double> get_reference_data() const;


private:
    ros::NodeHandle& nh;
    std::shared_ptr<DatabaseConfig>& databaseconfig;

    std::shared_ptr<Sensor> sensor;
    std::shared_ptr<ReferenceGoverner> referencegoverner;
    std::shared_ptr<PWM> pwm;

    
    std::vector<std::unique_ptr<Solver>> solvers;


};


#endif //POWERPACK_H
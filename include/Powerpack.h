#ifndef POWERPACK_H
#define POWERPACK_H

#include <ros/ros.h>

#include <iostream>
#include <memory>
#include <vector>

#include "ControlModule.h"
#include "DatabaseConfig.h"
#include "PWM.h"
#include "ReferenceGoverner.h"
#include "Sensor.h"

class Powerpack {
   public:
    Powerpack(ros::NodeHandle& nh,
              std::shared_ptr<DatabaseConfig>& databaseconfig);
    ~Powerpack();

    std::vector<double> get_sensor_data() const;
    std::vector<double> get_reference_data() const;

    void run();

   private:
    ros::NodeHandle& nh;
    std::shared_ptr<DatabaseConfig>& databaseconfig;

    std::shared_ptr<Sensor> sensor;
    std::shared_ptr<ReferenceGoverner> referencegoverner;
    std::shared_ptr<PWM> pwm;

    std::vector<std::unique_ptr<Solver>> solvers;
};

#endif  // POWERPACK_H
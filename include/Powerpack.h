#ifndef POWERPACK_H
#define POWERPACK_H

#include <ros/ros.h>

#include <iostream>
#include <memory>
#include <vector>

#include "DatabaseConfig.h"
#include "PWM.h"
#include "ReferenceGoverner.h"
#include "QP.h"
#include "Solver.h"
#include "Sensor.h"

class Powerpack {
   public:
    Powerpack(ros::NodeHandle& nh,
              std::shared_ptr<DatabaseConfig>& databaseconfig);
    ~Powerpack();

    std::vector<double> get_sensor_data() const;
    std::vector<double> get_reference_data() const;

    void set_pwm();

    void run();

   private:
    ros::NodeHandle& nh;
    std::shared_ptr<DatabaseConfig>& databaseconfig;

    std::shared_ptr<Sensor> sensor;
    std::shared_ptr<ReferenceGoverner> referencegoverner;
    std::shared_ptr<PWM> pwm;
    std::shared_ptr<QP> qp;

    std::vector<std::unique_ptr<Solver>> solvers;

    std::vector<double> pwm_values;
};

#endif  // POWERPACK_H
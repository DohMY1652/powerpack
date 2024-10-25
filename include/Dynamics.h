#ifndef DYNAMICS_H
#define DYNAMICS_H

#include <iostream>
#include <memory>
#include <vector>

#include "DatabaseConfig.h"

class Dynamics {
   public:
    Dynamics(std::shared_ptr<DatabaseConfig>& databaseconfig);
    ~Dynamics();

    void set_channel(int _channel);

    void calculate_valve_dynamic(double input, double pressure_in,
                                 double pressure_out, double volume);
    double get_flow_rate() const;
    double get_round_input() const;
    double get_round_pressure_in() const;
    double get_round_pressure_out() const;

   private:
    std::shared_ptr<DatabaseConfig>& databaseconfig;

    const double gas_constant = 0.287;  // J/g*K
    const double temperature = 293.15;  // K

    int channel;
    bool is_positive;
    std::vector<double> valve_const;
    double flow_rate;
    double round_input;
    double round_pressure_in;
    double round_pressure_out;
};

#endif  // DYNAMICS_H
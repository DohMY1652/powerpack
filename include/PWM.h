// PWM.h
#ifndef PWM_H
#define PWM_H

#include <iostream>
#include <vector>
#include <memory>

#include "DatabaseConfig.h"

class PWM {
public:
    PWM(std::shared_ptr<DatabaseConfig> &databaseconfig);
    ~PWM();

private:
    std::shared_ptr<DatabaseConfig> &databaseconfig;

    int n_channel;
    int frequency;
    int pid_pos_index;
    int pid_neg_index;

    std::vector<unsigned int> data;
};

#endif // PWM_H
#ifndef DYNAMICS_H
#define DYNAMICS_H

#include <iostream>
#include <vector>
#include <memory>

#include "DatabaseConfig.h"

class Dynamics {
public:
    Dynamics(std::shared_ptr<DatabaseConfig>& databaseconfig);
    ~Dynamics();

private:
    std::shared_ptr<DatabaseConfig> &databaseconfig;

};


#endif // DYNAMICS_H
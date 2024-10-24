#include <iostream>
#include <vector>

#include "Dynamics.h"
#include "DatabaseConfig.h"

Dynamics::Dynamics(std::shared_ptr<DatabaseConfig>& databaseconfig) 
 : databaseconfig(databaseconfig) {

}

Dynamics::~Dynamics() {
    
}
#ifndef QP_H
#define QP_H

#include <iostream>
#include <vector>
#include <memory>

#include "DatabaseConfig.h"

class QP {
public:
    QP(std::shared_ptr<DatabaseConfig>& databaseconfig);
    ~QP();

private:
    std::shared_ptr<DatabaseConfig> &databaseconfig;

};

#endif //QP_H
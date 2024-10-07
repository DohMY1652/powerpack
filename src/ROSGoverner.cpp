#include "ROSGoverner.h"

ROSGoverner::ROSGoverner() {
    ros::init(argc, argv, "Pos_neg_controller");
    ros::NodeHandle n;

}

ROSGoverner::~ROSGoverner() {

}


#ifndef SENSOR_H
#define SENSOR_H

#include <vector>

class Sensor {
    public:
        Sensor(int n_channel);
        ~Sensor();

        virtual void update();
        void print_all_data();
        double get_data(int index);
        int get_n_channel();
        

    protected:
        int n_channel;
        std::vector<double> data;
};



class ADS1263 : public Sensor {
    virtual void update() override;
};
#endif //SENSOR_H
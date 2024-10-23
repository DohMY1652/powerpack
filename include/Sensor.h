#ifndef SENSOR_H
#define SENSOR_H

#include <vector>

class Sensor {
    public:
        Sensor(int n_pos_channel,int n_neg_channel, std::vector<double> parameters);
        ~Sensor();

        void update(std::vector<double> _data);
        void print_all_data();
        double get_data(int index);

        std::vector<double> get_all_data();

        int get_n_channel();
        

    private:
        int n_pump_channel;
        int n_macro_channel;
        int n_pos_channel;
        int n_neg_channel;

        double pos_offset;
        double pos_gain;
        double neg_offset;
        double neg_gain;
        double atm_offset;

        std::vector<double> data;
};

#endif //SENSOR_H


// PWM.h
#ifndef PWM_H
#define PWM_H

#include <vector>

class PWM {
public:
    PWM(int n_channel);
    ~PWM();

    void set_data(unsigned int value, int PWM_idx);
    void update(std::vector<unsigned int> control_signal);
    void print_all_data();
    int get_n_channel();

    std::vector<unsigned int> get_control_signal();

private:
    int n_channel;
    std::vector<unsigned int> data;
};

#endif // PWM_H
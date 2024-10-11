#ifndef REFERENCEGOVERNER_H
#define REFERENCEGOVERNER_H

#include <vector>

class ReferenceGoverner{
    public:
        ReferenceGoverner(int n_channel, int id);
        ~ReferenceGoverner();

        void update(std::vector<double> _data);
        void update(std::vector<std::vector<double>> _data);
        void print_all_data();
        std::vector<double> get_data(int idx);

    private:
        int n_channel;
        int id;
        std::vector<std::vector<double>> data;

};

#endif //REFERENCE_GOVERNER_H
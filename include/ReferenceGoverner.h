#ifndef REFERENCEGOVERNER_H
#define REFERENCEGOVERNER_H

#include <vector>

class ReferenceGoverner{
    public:
        ReferenceGoverner(int n_channel, int id);
        ~ReferenceGoverner();

        void update(std::vector<double> _data);
        double get_data(int index);
        std::vector<double> get_all_data();

    private:
        int n_channel;
        int id;
        std::vector<double> data;

};

#endif //REFERENCE_GOVERNER_H
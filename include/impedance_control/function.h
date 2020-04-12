#ifndef DIFF_H
#define DIFF_H

class function {
    private:
        double x_[2];
        bool firstValue;
    public:
        function(void);
        void addValue(double value);
        double getValue(void);
        double diff(double sampling_time);
};

#endif
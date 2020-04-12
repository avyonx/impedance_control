#include <impedance_control/function.h>
#include <cmath>

function::function(void) :
    firstValue(true)
{
    for (int i = 0; i < 2; i++) {
        x_[i] = 0;
    }
}

void function::addValue(double value)
{
    if (firstValue) {
        firstValue = false;
        x_[1] = value;
        x_[0] = value;
    }
    else {
        x_[1] = x_[0];
        x_[0] = value;
    }
}

double function::getValue(void)
{
    return x_[0];
}

double function::diff(double sampling_time)
{
    double y;

    y = (x_[0] - x_[1]) / sampling_time;

    return y;
}
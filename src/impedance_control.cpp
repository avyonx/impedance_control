#include <impedance_control/impedance_control.h>
#include <math.h>

ImpedanceControl::ImpedanceControl(int rate):
    targetImpedanceType_(1),
    M_(0),
    B_(0),
    K_(0),
    dead_zone_(0),
    rate_(rate) {
}

void ImpedanceControl::setImpedanceFilterMass(float mass) {
    M_ = mass;
}

void ImpedanceControl::setImpedanceFilterDamping(float damping) {
    B_ = damping;
}

void ImpedanceControl::setImpedanceFilterStiffness(float stiffness) {
    K_ = stiffness;
}

void ImpedanceControl::setImpedanceFilterInitialValue(float initial_values) {
    int j;
    float y0[3], x0[3]; 

    for (j= 0; j < 3; j++) {
        y0[j] = initial_values;
        x0[j] = initial_values;
    }

    Gxr_.setInitialValues(y0, x0);
}

void ImpedanceControl::initializeImpedanceFilterTransferFunction(void) {
    float samplingTime;

    samplingTime = 1.0/rate_;

    Ge_.reset();
    Ge_.setNumerator(1.0, 0.0, 0.0);
    Ge_.setDenominator(K_, B_, M_);
    Ge_.c2d(samplingTime, "zoh");

    Gxr_.reset();
    if (targetImpedanceType_ == 1)
        Gxr_.setNumerator(K_, 0.0, 0.0);
    else if (targetImpedanceType_ == 2)
        Gxr_.setNumerator(K_, B_, 0.0);
    else if (targetImpedanceType_ == 3) 
        Gxr_.setNumerator(K_, B_, M_);
    Gxr_.setDenominator(K_, B_, M_);
    Gxr_.c2d(samplingTime, "zoh");
}

float ImpedanceControl::impedanceFilter(float e, float Xr) {
    float Xc;

    Xc = Ge_.getDiscreteOutput(deadZone(e, dead_zone_)) + Gxr_.getDiscreteOutput(Xr);

    return Xc;
}

void ImpedanceControl::setTargetImpedanceType(int type) {
    if (type < 1 || type > 3) targetImpedanceType_ = 1;
    else targetImpedanceType_ = type;
}

void ImpedanceControl::setDeadZone(float dead_zone) {
    dead_zone_ = dead_zone;
}

float ImpedanceControl::deadZone(float data, float limit) {
    float temp;

    if (fabs(data) < limit) {
        temp = 0.0;
    }
    else if (data > 0.0) {
        temp = data - limit;
    }
    else {
        temp = data + limit;
    }

    return temp;
}
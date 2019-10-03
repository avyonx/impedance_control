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

    Gxr_[0].setInitialValues(y0, x0);

    for (j= 0; j < 3; j++) {
        y0[j] = 0.0;
        x0[j] = 0.0;
    }
    Gxr_[1].setInitialValues(y0, x0);
    Gxr_[2].setInitialValues(y0, x0);

    for (j = 0; j< 3; j++)
    {
        Gvr_[j].setInitialValues(y0, x0);
        Gar_[j].setInitialValues(y0, x0);
    }
}

void ImpedanceControl::initializeImpedanceFilterTransferFunction(void) {
    float samplingTime;

    samplingTime = 1.0/rate_;

    //error transfer function
    Ge_[0].reset();
    Ge_[0].setNumerator(1.0, 0.0, 0.0);
    Ge_[0].setDenominator(K_, B_, M_);
    Ge_[0].c2d(samplingTime, "zoh");

    Ge_[1].reset();
    Ge_[1].setNumerator(0.0, 1.0, 0.0);
    Ge_[1].setDenominator(K_, B_, M_);
    Ge_[1].c2d(samplingTime, "zoh");

    Ge_[2].reset();
    Ge_[2].setNumerator(0.0, 0.0, 1.0);
    Ge_[2].setDenominator(K_, B_, M_);
    Ge_[2].c2d(samplingTime, "zoh");

    //Position transfer function
    Gxr_[0].reset();
    Gxr_[0].setNumerator(K_, 0.0, 0.0);
    Gxr_[0].setDenominator(K_, B_, M_);
    Gxr_[0].c2d(samplingTime, "zoh");

    Gxr_[1].reset();
    Gxr_[1].setNumerator(0.0, K_, 0.0);
    Gxr_[1].setDenominator(K_, B_, M_);
    Gxr_[1].c2d(samplingTime, "zoh");

    Gxr_[2].reset();
    Gxr_[2].setNumerator(0.0, 0.0, K_);
    Gxr_[2].setDenominator(K_, B_, M_);
    Gxr_[2].c2d(samplingTime, "zoh");

    //velocity transfer function
    Gvr_[0].reset();
    Gvr_[0].setNumerator(B_, 0.0, 0.0);
    Gvr_[0].setDenominator(K_, B_, M_);
    Gvr_[0].c2d(samplingTime, "zoh");

    Gvr_[1].reset();
    Gvr_[1].setNumerator(0.0, B_, 0.0);
    Gvr_[1].setDenominator(K_, B_, M_);
    Gvr_[1].c2d(samplingTime, "zoh");

    Gvr_[2].reset();
    Gvr_[2].setNumerator(0.0, 0.0, B_);
    Gvr_[2].setDenominator(K_, B_, M_);
    Gvr_[2].c2d(samplingTime, "zoh");

    //acceleration transfer function
    Gar_[0].reset();
    Gar_[0].setNumerator(M_, 0.0, 0.0);
    Gar_[0].setDenominator(K_, B_, M_);
    Gar_[0].c2d(samplingTime, "zoh");

    Gar_[1].reset();
    Gar_[1].setNumerator(0.0, M_, 0.0);
    Gar_[1].setDenominator(K_, B_, M_);
    Gar_[1].c2d(samplingTime, "zoh");

    Gar_[2].reset();
    Gar_[2].setNumerator(0.0, 0.0, M_);
    Gar_[2].setDenominator(K_, B_, M_);
    Gar_[2].c2d(samplingTime, "zoh");
}

void ImpedanceControl::impedanceFilter(float e, float Xr, float Vr, float Ar, float* X) {

    X[0] = Ge_[0].getDiscreteOutput(deadZone(e, dead_zone_)) + Gxr_[0].getDiscreteOutput(Xr) 
            + Gvr_[0].getDiscreteOutput(Vr) + Gar_[0].getDiscreteOutput(Ar);

    X[1] = Ge_[1].getDiscreteOutput(deadZone(e, dead_zone_)) + Gxr_[1].getDiscreteOutput(Xr) 
            + Gvr_[1].getDiscreteOutput(Vr) + Gar_[1].getDiscreteOutput(Ar);

    X[2] = Ge_[2].getDiscreteOutput(deadZone(e, dead_zone_)) + Gxr_[2].getDiscreteOutput(Xr) 
            + Gvr_[2].getDiscreteOutput(Vr) + Gar_[2].getDiscreteOutput(Ar);

    
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
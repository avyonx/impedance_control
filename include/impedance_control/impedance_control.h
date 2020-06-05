#ifndef IMPEDANCE_CONTROL_H
#define IMPEDANCE_CONTROL_H

#include <impedance_control/Tf2.h>

class ImpedanceControl {
private:
	double deadZone(double data, double limit);

	double K_, B_, M_, dead_zone_;
	double Xc_[3];
	int rate_;
	Tf2 Ge_[3], Gxr_[3], Gvr_[3], Gar_[3];
public:
	ImpedanceControl(int rate);
	void setImpedanceFilterMass(double mass);
	void setImpedanceFilterDamping(double damping);
	void setImpedanceFilterStiffness(double stiffness);
	void setImpedanceFilterInitialValue(double initial_values);
	void initializeImpedanceFilterTransferFunction(void);
	double* impedanceFilter(double f, double fd, double* Xr);
	void setDeadZone(double dead_zone);
};

#endif
#ifndef IMPEDANCE_CONTROL_H
#define IMPEDANCE_CONTROL_H

#include <impedance_control/Tf2.h>

class ImpedanceControl {
private:
	float deadZone(float data, float limit);

	float K_, B_, M_, dead_zone_;
	int targetImpedanceType_, rate_;
	Tf2 Ge_, Gxr_;
public:
	ImpedanceControl(int rate);
	void setImpedanceFilterMass(float mass);
	void setImpedanceFilterDamping(float damping);
	void setImpedanceFilterStiffness(float stiffness);
	void setImpedanceFilterInitialValue(float initial_values);
	void initializeImpedanceFilterTransferFunction(void);
	float impedanceFilter(float e, float Xr);
	void setTargetImpedanceType(int type);
	void setDeadZone(float dead_zone);
};


#endif
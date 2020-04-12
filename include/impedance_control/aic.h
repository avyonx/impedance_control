#ifndef AIC_H
#define AIC_H

#include <impedance_control/Tf1.h>
#include <impedance_control/Tf2.h>
#include <impedance_control/function.h>

class aic{
private:
	double deadZone(double data, double limit);
	void calculateAdaptiveEnvironmentStiffnessGainKp(double q);

	double sampling_time_, q_, kp_[3], xr_[3];
	double gamma_[2], wp_, wd_;
	double M_, B_, K_, dead_zone_;
	int rate_;
	Tf2 Gd2Kp_;
	Tf1 GdKp_, GKp_;
	function e_;
public:
	aic(int rate);
	void setAdaptiveParameterInitialValues(double kp0);
	void setAdaptiveParameters(double gamma1, double gamma2, double wp, double wd);
	void setImpedanceFilterParameters(double mass, double damping, double stiffness);
	void initializeAdaptationLaws(void);
	double* getAdaptiveEnvironmentStiffnessGainKp(void);
	double* compute(double f, double fd, double* Xd);
	void setDeadZone(double dead_zone);
	double getQ(void);
};

#endif
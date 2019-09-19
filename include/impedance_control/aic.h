#ifndef AIC_H
#define AIC_H

#include <impedance_control/Tf1.h>

class aic{
private:
	float calculateAdaptiveEnvironmentStiffnessGainKe(float e);

	float sampling_time_, ke0_, ke_, xr_, kp_;
	Tf1 Gke_;
public:
	aic(void);
	void setAdaptiveParameterInitialValues(float ke0);
	void initializeAdaptationLaws(float *k, float T);
	float getAdaptiveEnvironmentStiffnessGainKe(void);
	float compute(float e, float fd, float xd);
};

#endif
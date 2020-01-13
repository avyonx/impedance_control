#include <impedance_control/aic.h>

aic::aic(void):
	sampling_time_(1.0),
	ke0_(0.0),
	xr_(0.0),
	ke_(1.0),
	kp_(0.0) {

}

void aic::setAdaptiveParameterInitialValues(float ke0) {
	ke0_ = ke0;
}

void aic::initializeAdaptationLaws(float *k, float T) {
	sampling_time_ = T;

	Gke_.reset();
	Gke_.setNumerator(k[0], k[1]);
	Gke_.setDenominator(0.0, 1.0);
	Gke_.c2d(sampling_time_, "zoh");

	kp_ = k[2];
}

float aic::getAdaptiveEnvironmentStiffnessGainKe(void) {
	return ke_;
}

float aic::compute(float e, float fd, float xd) {
	ke_ = calculateAdaptiveEnvironmentStiffnessGainKe(e);

	xr_ = xd + fd*ke_*(1+e*kp_);

	return xr_;
}

float aic::calculateAdaptiveEnvironmentStiffnessGainKe(float e) {
	float ke;

	ke = ke0_ + Gke_.getDiscreteOutput(e);
	
	return ke;
}


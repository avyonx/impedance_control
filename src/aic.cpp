#include <impedance_control/aic.h>
#include <math.h>

aic::aic(int rate):
	sampling_time_(1.0),
	wp_(1.0),
	wd_(0.0),
	M_(0.0),
	B_(0.0),
	K_(0.0),
	q_(0.0),
	dead_zone_(0.0),
	rate_(rate)
{
	gamma_[0] = 0.0;
	gamma_[1] = 0.0;

	xr_[0] = 0.0;
	xr_[1] = 0.0;
	xr_[2] = 0.0;

	kp_[0] = 0.0;
	kp_[1] = 0.0;
	kp_[2] = 0.0;
}

void aic::setAdaptiveParameterInitialValues(double kp0) {
	double y[2], x[2];

	y[0] = kp0;
	y[1] = kp0;

	x[0] = 0.0;
	x[1] = 0.0;

	GKp_.setInitialValues(y, x);
	kp_[0] = kp0;
	kp_[1] = 0.0;
	kp_[2] = 0.0;
}

void aic::setImpedanceFilterParameters(double mass, double damping, double stiffness)
{
	M_ = mass;
	B_ = damping;
	K_ = stiffness;
}

void aic::setAdaptiveParameters(double gamma1, double gamma2, double wp, double wd)
{
	gamma_[0] = gamma1;
	gamma_[1] = gamma2;
	wp_ = wp;
	wd_ = wd;
}

void aic::initializeAdaptationLaws(void) {
	sampling_time_ = 1.0 / rate_;

	Gd2Kp_.reset();
	Gd2Kp_.setNumerator(0.0, M_*gamma_[0], M_*gamma_[1]);
	Gd2Kp_.setDenominator(K_, B_, M_);
	Gd2Kp_.c2d(sampling_time_, "tustin");

	GdKp_.reset();
	GdKp_.setNumerator(1.0, 0.0);
	GdKp_.setDenominator(0.0, 1.0);
	GdKp_.c2d(sampling_time_, "zoh");

	GKp_.reset();
	GKp_.setNumerator(1.0, 0.0);
	GKp_.setDenominator(0.0, 1.0);
	GKp_.c2d(sampling_time_, "zoh");
}

double* aic::getAdaptiveEnvironmentStiffnessGainKp(void) {
	return kp_;
}

double aic::getQ(void) {
	return q_;
}

void aic::setDeadZone(double dead_zone) {
	dead_zone_ = dead_zone;
}

double* aic::compute(double f, double fd, double* Xd) {
	double e = fd - f;
	e_.addValue(e);

	if (fd != 0.0)
	{
		q_ = (wp_ * e_.getValue() + wd_ * e_.diff(sampling_time_)) / fd;
		calculateAdaptiveEnvironmentStiffnessGainKp(q_);
	}

	xr_[0] = Xd[0] + kp_[0] * fd;
	xr_[1] = Xd[1] + kp_[1] * fd;
	xr_[2] = Xd[2] + kp_[2] * fd;

	return xr_;
}

void aic::calculateAdaptiveEnvironmentStiffnessGainKp(double q) {

	kp_[2] = Gd2Kp_.getDiscreteOutput(q);
	kp_[1] = GdKp_.getDiscreteOutput(kp_[2]);
	kp_[0] = GKp_.getDiscreteOutput(kp_[1]);
}

double aic::deadZone(double data, double limit) {
	double temp;

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


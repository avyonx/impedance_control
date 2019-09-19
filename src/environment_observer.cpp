#include <impedance_control/environment_observer.h>

EnvironmentObserver::EnvironmentObserver(void) {
	P_.setZero();
	V_.setZero();
	W_.setZero();
	xa_hat_.setZero();
	Ga_.setIdentity();
	xa_hat_pred_.setZero();
	y_pred_.setZero();
	P_pred_.setZero();
	
	// 1c.Output estimate.
	// Observer matrix.
	Ca_.setZero();
	Ca_ << 1, 0, 0, 0, 0, 0, 0, 1;

	// Measurement matrix.
	Ha_.setIdentity();
}

void EnvironmentObserver::initStateVector(double dxe0, double Ke0, double De0, double f0) {
	xa_hat_ << dxe0, Ke0, De0, f0;
}

void EnvironmentObserver::initCovValues(double P_dxe_INIT, double P_Ke_INIT, double P_De_INIT,
	double P_fe_INIT, double v_dxe, double v_Ke, double v_De, double v_fe, double w_dxe, double w_fe)
{
	P_.diagonal() << pow(P_dxe_INIT, 2), pow(P_Ke_INIT, 2), pow(P_De_INIT, 2), pow(P_fe_INIT, 2);
	V_.diagonal() << pow(v_dxe, 2), pow(v_Ke, 2), pow(v_De, 2), pow(v_fe, 2);
	W_.diagonal() << pow(w_dxe, 2), pow(w_fe, 2);
}

void EnvironmentObserver::modelUpdate(double dT) {
	double dxe_old, Ke_old, De_old, fe_old, De_old_inv;
	double dxe_pred, Ke_pred, De_pred, fe_pred;

	Eigen::Matrix<double, NUM_PROCES_NOISE_STATES, NUM_PROCES_NOISE_STATES> Aa;
	Eigen::Matrix<double, NUM_COV_MAT_STATES, NUM_COV_MAT_STATES> Q;
	Aa.setZero();

	dxe_old = xa_hat_(0, 0);
	Ke_old = xa_hat_(1, 0);
	De_old = xa_hat_(2, 0);
	fe_old = xa_hat_(3, 0);

	De_old_inv = 1.0 / (De_old);


	// EKF steps.
	// 0. Differentiation matrices.
	// Step 0: Compute Aa, Ga.
	Aa << (1 - De_old_inv * Ke_old * dT), -De_old_inv * (dxe_old)* dT, -1 * (fe_old * dT - Ke_old * dxe_old * dT) / pow(De_old, 2.0), De_old_inv * dT,
		0, 1, 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1;

	// 1. PREDICTION.
	// 1a.State estimate time update.
	dxe_pred = dxe_old * (1 - De_old_inv * Ke_old * dT) + De_old_inv * fe_old * dT;
	Ke_pred = Ke_old;
	De_pred = De_old;
	fe_pred = fe_old;

	xa_hat_pred_ << dxe_pred, Ke_pred, De_pred, fe_pred;

	Q = Ga_ * V_ * Ga_.transpose();
	P_pred_ = Aa * P_ * Aa.transpose() + Q;


	// Output equation.
	y_pred_ = Ca_ * xa_hat_pred_;

}

void EnvironmentObserver::measureUpdate(double dxe, double fe) {
	Eigen::Matrix<double, NUM_MEASURE_NOISE_STATES, NUM_MEASURE_NOISE_STATES> R;
	Eigen::Matrix<double, NUM_MEASURE_NOISE_STATES, 1> y_meas;
	Eigen::Matrix<double, NUM_PROCES_NOISE_STATES, NUM_MEASURE_NOISE_STATES> K_EKF;

	// 2. CORRECTION.
	//2a.Kalman gain.
	R = Ha_ * W_ * Ha_.transpose();
	K_EKF = (P_pred_ * Ca_.transpose()) * (Ca_ * P_pred_ * Ca_.transpose() + R).inverse();

	// 2b State estimate measurement update.
	// "Measurement".
	y_meas << dxe, fe;

	
	xa_hat_ = xa_hat_pred_ + K_EKF * (y_meas - y_pred_);

	// 2c Error covariance measurement update.
	P_ = P_pred_ - K_EKF * Ca_ * P_pred_;
}
#pragma once

#include <eigen3/Eigen/Eigen>
#include <math.h> 

constexpr int NUM_COV_MAT_STATES = 4;
constexpr int NUM_PROCES_NOISE_STATES = 4;
constexpr int NUM_MEASURE_NOISE_STATES = 2;

class EnvironmentObserver
{

public:
	EnvironmentObserver();
	void modelUpdate(double dT);
	void measureUpdate(double dxe, double fe);
	void initStateVector(double dxe0, double Ke0, double De0, double f0);
	void initCovValues(double P_dxe_INIT = 1, double P_Ke_INIT = 10000, 
		double P_De_INIT = 1, double P_fe_INIT = 10, double v_dxe = 10,
		double v_Ke = 100, double v_De = 0.1, double v_fe = 0.1,
		double w_dxe = 0.01, double w_fe = 0.001);

	Eigen::Matrix<double, NUM_PROCES_NOISE_STATES, 1> getXa() {
		return xa_hat_;
	};

	Eigen::Matrix<double, NUM_COV_MAT_STATES, NUM_COV_MAT_STATES> getP() {
		return P_;
	};

	double getDXe(void) {
		return xa_hat_(0, 0);
	};

	double getKe(void) {
		return xa_hat_(1, 0);
	};

	double getDe(void) {
		return xa_hat_(2, 0);
	};

	double getFe(void) {
		return xa_hat_(3, 0);
	};

private:
	Eigen::Matrix<double, NUM_COV_MAT_STATES, NUM_COV_MAT_STATES> P_;
	Eigen::DiagonalMatrix<double, NUM_PROCES_NOISE_STATES> V_;
	Eigen::DiagonalMatrix<double, NUM_MEASURE_NOISE_STATES> W_;
	Eigen::Matrix<double, NUM_PROCES_NOISE_STATES, 1> xa_hat_;
	Eigen::Matrix<double, NUM_PROCES_NOISE_STATES, NUM_PROCES_NOISE_STATES> Ga_;
	Eigen::Matrix<double, NUM_PROCES_NOISE_STATES, 1> xa_hat_pred_;
	Eigen::Matrix<double, NUM_MEASURE_NOISE_STATES, NUM_PROCES_NOISE_STATES > Ca_;
	Eigen::Matrix<double, NUM_MEASURE_NOISE_STATES, NUM_MEASURE_NOISE_STATES> Ha_;
	Eigen::Matrix<double, NUM_MEASURE_NOISE_STATES, 1> y_pred_;
	Eigen::Matrix<double, NUM_PROCES_NOISE_STATES, NUM_PROCES_NOISE_STATES> P_pred_;
};
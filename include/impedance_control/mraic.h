#ifndef MRAIC_H
#define MRAIC_H

#include <impedance_control/diff2.h>
#include <impedance_control/Tf1.h>
#include <impedance_control/MRAIController.h>
#include <impedance_control/median_filter.h>
#include <stdint.h>

class mraic{
	private:
		void setReferenceModelInitialConditions(float ym0, float dym0);
		float calculateAdaptiveProportionalGainKp(float q, float e, float de);
		float calculateAdaptiveDerivativeGainKd(float q, float e, float de);
		float calculateReferencePositionSignal(float q);

		float time_, ym0_, dym0_, samplingTime_;
		float a_[2], b_[2], c_[2], sigma_[2];
		float g0_, kp0_, kd0_, wp_, wd_;
		float kp_, kd_, g_, e_[2], de_;
		float q_, ym_[2], u_;
		uint8_t rm_type_;
		bool reference_model_init_, impact_;
		median_filter error_median;
		diff2 Ym_;
		Tf1 Gg, Gd, Gp;


	public:
		mraic(void);
		void initializeReferenceModel(float zeta, float omega);
		void setImpact(bool impact);
		void setAdaptiveParameterInitialValues(float g0, float kp0, float kd0);
		void initializeAdaptationLaws(float *a, float *b, float *c, float *sigma, float T);
		float getAdaptiveProportionalGainKp(void);
		float getAdaptiveDerivativeGainKd(void);
		float getReferencePositionSignal(void);
		void create_msg(impedance_control::MRAIController &msg);
		void setWeightingFactors(float wp, float wd);
		float compute(float dt, float e);
};

#endif
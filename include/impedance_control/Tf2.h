#ifndef TF2_H
#define TF2_H

#include <string>

class Tf2{
	private:
		enum PoleType { DOUBLE_REAL, SINGLE_REAL};

		bool zohTransform(void);
		bool tustinTransform(void);
		double d0_, d1_, d2_;
		double n0_, n1_, n2_;
		double dc0_, dc1_, dc2_;
		double nc0_, nc1_, nc2_;
		double T_, a_, b_;
		double x_[3];
		double y_[3];
		Tf2::PoleType poletype_;
		bool numeratorInit_, denominatorInit_;

	public:
		Tf2(void);
		bool setDenominator(double d0, double d1, double d2);
		bool setNumerator(double n0, double n1, double n2);
		double getDiscreteOutput(double input);
		void setSamplingTime(double samplingTime);
		bool c2d(double samplingTime, std::string method);
		void getDiscreteDenominator(double *d0, double *d1, double *d2);
		void getDiscreteNumerator(double *n0, double *n1, double *n2);
		void setInitialValues(double *y0, double *x0);
		void reset(void);
	
};

#endif
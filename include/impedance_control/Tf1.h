#ifndef TF1_H
#define TF1_H

#include <string>

class Tf1{
	private:
		bool zohTransform(void);
		bool tustinTransform(void);
		double d0_, d1_;
        double n0_, n1_;
        double dc0_, dc1_;
        double nc0_, nc1_;
        double T_;
        double x_[2];
        double y_[2];
		bool numeratorInit_, denominatorInit_;

	public:
		Tf1(void);
		bool setDenominator(double d0, double d1);
		bool setNumerator(double n0, double n1);
		double getDiscreteOutput(double input);
		void setSamplingTime(double samplingTime);
		bool c2d(double samplingTime, std::string method);
		void getDiscreteDenominator(double *d0, double *d1);
		void getDiscreteNumerator(double *n0, double *n1);
		void setInitialValues(double *y0, double *x0);
		void reset(void);
	
};

#endif
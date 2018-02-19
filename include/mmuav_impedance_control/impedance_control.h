#ifndef MMUAV_FORCE_CONTROL_H
#define MMUAV_FORCE_CONTROL_H

#include "ros/ros.h"
#include <mmuav_impedance_control/Tf2.h>
#include <mmuav_impedance_control/median_filter.h>
#include <mmuav_impedance_control/diff2.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>
#include <mmuav_impedance_control/mraic.h>


class ImpedanceControl{
	private:
		void force_measurement_cb(const geometry_msgs::WrenchStamped &msg);
		void pose_ref_cb(const geometry_msgs::PoseStamped &msg);
		void force_torque_cb(const geometry_msgs::WrenchStamped &msg);
		void initializeImpedanceFilterTransferFunction(void);
		float getFilteredForceZ(void);
		float getFilteredForceX(void);
		float getFilteredForceY(void);
		float getFilteredTorqueX(void);
		float getFilteredTorqueY(void);
		float getFilteredTorqueZ(void);
		bool check_collision(void);
		bool check_impact(void);
		float* impedanceFilter(float *e, float *Xr);
		float* modelReferenceAdaptiveImpedanceControl(float dt, float *e, float *g0);
		void quaternion2euler(float *quaternion, float *euler);
		void initializeMRACControl(void);

		volatile bool start_flag_, force_sensor_calibration_flag_;
		bool impact_flag_, collision_;
		float *force_x_meas_;
		float *force_z_meas_;
		float *force_y_meas_;
		float *torque_x_meas_;
		float *torque_y_meas_;
		float *torque_z_meas_;
		float M_[6], B_[6], K_[6], omega_[6], zeta_[6], kd0_[6], sigma3_[6];
		float em0_[6], dem0_[6], wp_[6], wd_[6], fe_[6], kp0_[6], sigma2_[6];
		float a1_[6], b1_[6], c1_[6], a2_[6], b2_[6], c2_[6], sigma1_[6];
		float force_z_offset_, force_y_offset_, force_x_offset_, mrac_time_;
		float torque_y_offset_, torque_x_offset_, torque_z_offset_;
		int rate_, moving_average_sample_number_, targetImpedanceType_;

		median_filter force_z_med_filt, force_x_med_filt, force_y_med_filt;

		geometry_msgs::PoseStamped pose_ref_;
		geometry_msgs::WrenchStamped force_torque_ref_;
		std_msgs::Float64 yaw_ref_;
		std::string x_c_ros_topic_;

		ros::NodeHandle n_;

		ros::Subscriber force_ros_sub_;
		ros::Subscriber force_torque_ref_ros_sub_, pose_ref_ros_sub_;

		ros::Publisher force_filtered_pub_, pose_commanded_pub_;

		Tf2 Ge_[6], Gxr_[6];
		mraic mraic_[6];

	public:
		ImpedanceControl(int rate, int moving_average_sample_number, int median_filter_size);
		~ImpedanceControl();
		void setImpedanceFilterMass(float *mass);
		void setImpedanceFilterDamping(float *damping);
		void setImpedanceFilterStiffness(float *stiffness);
		void setTargetImpedanceType(int type);
		void LoadImpedanceControlParameters(std::string file);
		void run();
};

#endif
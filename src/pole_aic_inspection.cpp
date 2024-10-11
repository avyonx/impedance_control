#include <impedance_control/impedance_control.h>
#include <impedance_control/aic.h>

#include <ros/package.h>
#include "ros/ros.h"
#include <dynamic_reconfigure/server.h>

#include <rosgraph_msgs/Clock.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/WrenchStamped.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_srvs/SetBool.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>
#include <impedance_control/ImpedanceControlConfig.h>

#include "yaml-cpp/yaml.h"
#include <tf2/LinearMath/Quaternion.h>

#include <vector>
#include <sstream>


const auto getYaw = [](double qx, double qy, double qz, double qw)
{
	return atan2(2 * (qw * qz + qx * qy), qw * qw + qx * qx - qy * qy - qz * qz);
};

const auto calculateQuaternion = [](double heading)
{
	tf2::Quaternion q;
	q.setRPY(0, 0, heading);

	geometry_msgs::Quaternion q_msg;
	q_msg.x = q.getX();
	q_msg.y = q.getY();
	q_msg.z = q.getZ();
	q_msg.w = q.getW();
	return q_msg;
};

class ImpedanceControlNode {
private:

// init from imports
	void initializeImpedanceFilterTransferFunction() {
		impedance_control_x_.initializeImpedanceFilterTransferFunction();
		impedance_control_y_.initializeImpedanceFilterTransferFunction();
		impedance_control_z_.initializeImpedanceFilterTransferFunction();
	};

	void setImpedanceFilterInitialValue(double *initial_values) {
		impedance_control_x_.setImpedanceFilterInitialValue(initial_values[0]);
		impedance_control_y_.setImpedanceFilterInitialValue(initial_values[1]);
		impedance_control_z_.setImpedanceFilterInitialValue(initial_values[2]);
	};

	void initializeAdaptationLaws()
	{
		aic_control_x_.initializeAdaptationLaws();
		aic_control_y_.initializeAdaptationLaws();
		aic_control_z_.initializeAdaptationLaws();
	};

	void setAdaptiveEnvironmentStiffnessInitialValue()
	{
		aic_control_x_.setAdaptiveParameterInitialValues(kp0_[0]);
		aic_control_y_.setAdaptiveParameterInitialValues(kp0_[1]);
		aic_control_z_.setAdaptiveParameterInitialValues(kp0_[2]);
	};

	bool simulation_flag_, force_msg_received_, reconfigure_start_, trajectory_msg_received_;
	bool pose_meas_received_, impedance_start_flag_, carrot_in_hold_mode_;
	double *xr_, *xc_, *yr_, *yc_, *zr_, *zc_, *xKp_, *yKp_, *zKp_;
	double qxc_[3], qyc_[3], qzc_[3], qwc_[3];
	double xq_, yq_, zq_;
	int rate_;

	std::vector<double> kp1_, kp2_, wp_, wd_, M_, B_, K_, dead_zone_, kp0_;

	// vars for sub msgs
	rosgraph_msgs::Clock clock_;
	geometry_msgs::WrenchStamped force_meas_, force_torque_ref_;
	geometry_msgs::PoseStamped pose_ref_, pose_meas_;
	geometry_msgs::Twist vel_ref_, acc_ref_;

	ImpedanceControl impedance_control_z_, impedance_control_y_, impedance_control_x_;
	aic aic_control_z_, aic_control_x_, aic_control_y_; 
public:
	ImpedanceControlNode(int rate, bool simulation):
		rate_(rate),
		simulation_flag_(simulation),
		force_msg_received_(true),
		trajectory_msg_received_(false),
		pose_meas_received_(false),
		impedance_start_flag_(false),
		carrot_in_hold_mode_(false),
		impedance_control_x_(rate),
		impedance_control_y_(rate),
		impedance_control_z_(rate),
		aic_control_x_(rate),
		aic_control_y_(rate),
		aic_control_z_(rate),
		reconfigure_start_(false) {
	};

	void clockCb(const rosgraph_msgs::Clock &msg) {
	    clock_ = msg;
	};

	// force measurement feedback
	void forceMeasurementCb(const geometry_msgs::WrenchStamped &msg) {
		force_msg_received_ = true;
		force_meas_ = msg;
	};

	// pose reference to be removed
	void poseRefCb(const geometry_msgs::PoseStamped &msg) {
	    pose_ref_ = msg;
	    vel_ref_ = geometry_msgs::Twist();
	    acc_ref_ = geometry_msgs::Twist();
		trajectory_msg_received_ = true;
	};

	//subscriber for status of carrot
	void carrotStatusCb(const std_msgs::String &msg) {
		if (msg.data == "HOLD") {
			carrot_in_hold_mode_ = true;
			// ROS_INFO("Carrot is in HOLD mode.");
		} else {
			carrot_in_hold_mode_ = false;
			// ROS_INFO("Carrot is NOT in HOLD mode.");
		}
	};

	// array of trajectory points
	void trajectoryRefCb(const trajectory_msgs::MultiDOFJointTrajectory &msg) {
		if (msg.points.size() > 0)
		{
			pose_ref_.header = msg.header;
			pose_ref_.pose.position.x = msg.points[0].transforms[0].translation.x;
			pose_ref_.pose.position.y = msg.points[0].transforms[0].translation.y;
			pose_ref_.pose.position.z = msg.points[0].transforms[0].translation.z;
			pose_ref_.pose.orientation.x = msg.points[0].transforms[0].rotation.x;
			pose_ref_.pose.orientation.y = msg.points[0].transforms[0].rotation.y;
			pose_ref_.pose.orientation.z = msg.points[0].transforms[0].rotation.z;
			pose_ref_.pose.orientation.w = msg.points[0].transforms[0].rotation.w;
			vel_ref_.linear.x = msg.points[0].velocities[0].linear.x;
			vel_ref_.linear.y = msg.points[0].velocities[0].linear.y;
			vel_ref_.linear.z = msg.points[0].velocities[0].linear.z;
			vel_ref_.angular.x = msg.points[0].velocities[0].angular.x;
			vel_ref_.angular.y = msg.points[0].velocities[0].angular.y;
			vel_ref_.angular.z = msg.points[0].velocities[0].angular.z;
			acc_ref_.linear.x = msg.points[0].accelerations[0].linear.x;
			acc_ref_.linear.y = msg.points[0].accelerations[0].linear.y;
			acc_ref_.linear.z = msg.points[0].accelerations[0].linear.z;
			acc_ref_.angular.x = msg.points[0].accelerations[0].angular.x;
			acc_ref_.angular.y = msg.points[0].accelerations[0].angular.y;
			acc_ref_.angular.z = msg.points[0].accelerations[0].angular.z;
			
		} 
		
	 	trajectory_msg_received_ = true;		

	 	// ROS_INFO("************************");
     	// ROS_INFO("Xe(x): %f", pose_ref_.pose.position.x);
     	// ROS_INFO("Xe(y): %f", pose_ref_.pose.position.y);
     	// ROS_INFO("Xe(z): %f", pose_ref_.pose.position.z);
     	// ROS_INFO("************************");
	 }

	// main sub to trajectory
	void trajectoryPointRefCb(const trajectory_msgs::MultiDOFJointTrajectoryPoint &msg) {
		pose_ref_.pose.position.x = msg.transforms[0].translation.x;
		pose_ref_.pose.position.y = msg.transforms[0].translation.y;
		pose_ref_.pose.position.z = msg.transforms[0].translation.z;
		pose_ref_.pose.orientation.x = msg.transforms[0].rotation.x;
		pose_ref_.pose.orientation.y = msg.transforms[0].rotation.y;
		pose_ref_.pose.orientation.z = msg.transforms[0].rotation.z;
		pose_ref_.pose.orientation.w = msg.transforms[0].rotation.w;

		vel_ref_.linear.x = msg.velocities[0].linear.x;
		vel_ref_.linear.y = msg.velocities[0].linear.y;
		vel_ref_.linear.z = msg.velocities[0].linear.z;
		vel_ref_.angular.x = msg.velocities[0].angular.x;
		vel_ref_.angular.y = msg.velocities[0].angular.y;
		vel_ref_.angular.z = msg.velocities[0].angular.z;
	
		acc_ref_.linear.x = msg.accelerations[0].linear.x;
		acc_ref_.linear.y = msg.accelerations[0].linear.y;
		acc_ref_.linear.z = msg.accelerations[0].linear.z;
		acc_ref_.angular.x = msg.accelerations[0].angular.x;
		acc_ref_.angular.y = msg.accelerations[0].angular.y;
		acc_ref_.angular.z = msg.accelerations[0].angular.z;

		trajectory_msg_received_ = true;

	}

	void carrotMeasOdomCb(const trajectory_msgs::MultiDOFJointTrajectoryPoint &msg) {

		pose_meas_received_ = true;

		pose_meas_.pose.position.x = msg.transforms[0].translation.x;
		pose_meas_.pose.position.y = msg.transforms[0].translation.y;
		pose_meas_.pose.position.z = msg.transforms[0].translation.z;
		pose_meas_.pose.orientation.x = msg.transforms[0].rotation.x;
		pose_meas_.pose.orientation.y = msg.transforms[0].rotation.y;
		pose_meas_.pose.orientation.z = msg.transforms[0].rotation.z;
		pose_meas_.pose.orientation.w = msg.transforms[0].rotation.w;

		// rest of trajectory msg

		// vel_meas_.linear.x = msg.velocities[0].linear.x;
		// vel_meas_.linear.y = msg.velocities[0].linear.y;
		// vel_meas_.linear.z = msg.velocities[0].linear.z;
		// vel_meas_.angular.x = msg.velocities[0].angular.x;
		// vel_meas_.angular.y = msg.velocities[0].angular.y;
		// vel_meas_.angular.z = msg.velocities[0].angular.z;
	
		// acc_meas_.linear.x = msg.accelerations[0].linear.x;
		// acc_meas_.linear.y = msg.accelerations[0].linear.y;
		// acc_meas_.linear.z = msg.accelerations[0].linear.z;
		// acc_meas_.angular.x = msg.accelerations[0].angular.x;
		// acc_meas_.angular.y = msg.accelerations[0].angular.y;
		// acc_meas_.angular.z = msg.accelerations[0].angular.z;

	}

	// sub to force reference
	void forceTorqueRefCb(const geometry_msgs::WrenchStamped &msg) {
    	force_torque_ref_ = msg;
	};

	// odometry of drone - might stay
	void poseMeasOdomCb(const nav_msgs::Odometry &msg) {
		pose_meas_received_ = true;
		pose_meas_.header = msg.header;
		pose_meas_.pose = msg.pose.pose;
	};

	// duplicate from before can be removed
	void poseMeasPoseStampedCb(const geometry_msgs::PoseStamped &msg) {
		pose_meas_received_ = true;
		pose_meas_ = msg;
	};

	// Adaptation of AIC and Impedance Control parameters
	void reconfigureCb(impedance_control::ImpedanceControlConfig &config, uint32_t level) {

	    if (!reconfigure_start_) {
	        // Initial parameter setting for AIC and Impedance Control (X, Y, Z axes)
	
	        // AIC Parameters
	        config.kp1_x = kp1_[0];
	        config.kp2_x = kp2_[0];
	        config.wp_x = wp_[0];
	        config.wd_x = wd_[0];
	
	        config.kp1_y = kp1_[1];
	        config.kp2_y = kp2_[1];
	        config.wp_y = wp_[1];
	        config.wd_y = wd_[1];

	        config.kp1_z = kp1_[2];
	        config.kp2_z = kp2_[2];
	        config.wp_z = wp_[2];
	        config.wd_z = wd_[2];
	
	        // Impedance Control Parameters
	        // config.M_x = M_[0];
	        // config.B_x = B_[0];
	        // config.K_x = K_[0];
	        // config.dead_zone_x = dead_zone_[0];

	        // config.M_y = M_[1];
	        // config.B_y = B_[1];
	        // config.K_y = K_[1];
	        // config.dead_zone_y = dead_zone_[1];
 
	        // config.M_z = M_[2];
	        // config.B_z = B_[2];
	        // config.K_z = K_[2];
	        // config.dead_zone_z = dead_zone_[2];

	        reconfigure_start_ = true;
	    } else {
	        // Updating AIC and Impedance Control parameters based on dynamic reconfigure input

	        // AIC Parameters
	        kp1_[0] = config.kp1_x;
	        kp2_[0] = config.kp2_x;
	        wp_[0] = config.wp_x;
	        wd_[0] = config.wd_x;

	        kp1_[1] = config.kp1_y;
	        kp2_[1] = config.kp2_y;
	        wp_[1] = config.wp_y;
	        wd_[1] = config.wd_y;

	        kp1_[2] = config.kp1_z;
	        kp2_[2] = config.kp2_z;
	        wp_[2] = config.wp_z;
	        wd_[2] = config.wd_z;

	        aic_control_x_.setAdaptiveParameters(kp1_[0], kp2_[0], wp_[0], wd_[0]);
	        aic_control_y_.setAdaptiveParameters(kp1_[1], kp2_[1], wp_[1], wd_[1]);
	        aic_control_z_.setAdaptiveParameters(kp1_[2], kp2_[2], wp_[2], wd_[2]);

	        // Impedance Control Parameters
	        // M_[0] = config.M_x;
			// ROS_INFO("M_x = %.2f", M_[0]);
	        // B_[0] = config.B_x;
			// ROS_INFO("B_x = %.2f", B_[0]);
	        // K_[0] = config.K_x;
			// ROS_INFO("K_x = %.2f", B_[0]);
	        // dead_zone_[0] = config.dead_zone_x;

	        // M_[1] = config.M_y;
			// ROS_INFO("M_y = %.2f", M_[1]);
	        // B_[1] = config.B_y;
			// ROS_INFO("B_y = %.2f", B_[1]);
	        // K_[1] = config.K_y;
			// ROS_INFO("K_y = %.2f", K_[1]);
	        // dead_zone_[1] = config.dead_zone_y;

	        // M_[2] = config.M_z;
			// ROS_INFO("M_Z = %.2f", M_[2]);
	        // B_[2] = config.B_z;
			// ROS_INFO("B_z = %.2f", B_[2]);
	        // K_[2] = config.K_z;
			// ROS_INFO("K_z = %.2f", K_[2]);
	        // dead_zone_[2] = config.dead_zone_z;

	        impedance_control_x_.setImpedanceFilterMass(M_[0]);
	        impedance_control_x_.setImpedanceFilterDamping(B_[0]);
	        impedance_control_x_.setImpedanceFilterStiffness(K_[0]);
	        impedance_control_x_.setDeadZone(dead_zone_[0]);

	        impedance_control_y_.setImpedanceFilterMass(M_[1]);
	        impedance_control_y_.setImpedanceFilterDamping(B_[1]);
	        impedance_control_y_.setImpedanceFilterStiffness(K_[1]);
	        impedance_control_y_.setDeadZone(dead_zone_[1]);

	        impedance_control_z_.setImpedanceFilterMass(M_[2]);
	        impedance_control_z_.setImpedanceFilterDamping(B_[2]);
	        impedance_control_z_.setImpedanceFilterStiffness(K_[2]);
	        impedance_control_z_.setDeadZone(dead_zone_[2]);

	        initializeAdaptationLaws();
	    }
	};

	std::string vectorToString(const std::vector<double>& vec) {
    std::stringstream ss;
    ss << "[";
    for (size_t i = 0; i < vec.size(); ++i) {
        ss << vec[i];
        if (i != vec.size() - 1) {
            ss << ", ";
        }
    }
    ss << "]";
    return ss.str();
	}

	
	// Load Impedance and AIC control parameters from a YAML file and update dynamic reconfigure
	void loadImpedanceControlParameters(std::string file) {
    YAML::Node config = YAML::LoadFile(file);

	    // Load parameters from YAML
	    M_ = config["IMPEDANCE_FILTER"]["M"].as<std::vector<double>>();
	    B_ = config["IMPEDANCE_FILTER"]["B"].as<std::vector<double>>();
	    K_ = config["IMPEDANCE_FILTER"]["K"].as<std::vector<double>>();
	    dead_zone_ = config["IMPEDANCE_FILTER"]["dead_zone"].as<std::vector<double>>();

	    kp1_ = config["AIC"]["INTEGRAL_ADAPTATION_GAINS"]["ke1"].as<std::vector<double>>();
	    kp2_ = config["AIC"]["PROPORTIONAL_ADAPTATION_GAINS"]["ke2"].as<std::vector<double>>();
	    kp0_ = config["AIC"]["INITIAL_GAINS"]["Ke"].as<std::vector<double>>();
	    wp_ = config["AIC"]["WEIGHTING_FACTORS"]["Wp"].as<std::vector<double>>();
	    wd_ = config["AIC"]["WEIGHTING_FACTORS"]["Wd"].as<std::vector<double>>();

	    // Log loaded parameters
	    ROS_INFO_STREAM("IMPEDANCE_FILTER M: " << vectorToString(M_));
	    ROS_INFO_STREAM("IMPEDANCE_FILTER B: " << vectorToString(B_));
	    ROS_INFO_STREAM("IMPEDANCE_FILTER K: " << vectorToString(K_));
	    ROS_INFO_STREAM("IMPEDANCE_FILTER dead_zone: " << vectorToString(dead_zone_));

	    ROS_INFO_STREAM("AIC INTEGRAL_ADAPTATION_GAINS ke1: " << vectorToString(kp1_));
	    ROS_INFO_STREAM("AIC PROPORTIONAL_ADAPTATION_GAINS ke2: " << vectorToString(kp2_));
	    ROS_INFO_STREAM("AIC INITIAL_GAINS Ke: " << vectorToString(kp0_));
	    ROS_INFO_STREAM("AIC WEIGHTING_FACTORS Wp: " << vectorToString(wp_));
	    ROS_INFO_STREAM("AIC WEIGHTING_FACTORS Wd: " << vectorToString(wd_));

	    // Set parameters for each axis (X, Y, Z) in the control system
	    impedance_control_x_.setImpedanceFilterMass(M_[0]);
	    impedance_control_x_.setImpedanceFilterDamping(B_[0]);
	    impedance_control_x_.setImpedanceFilterStiffness(K_[0]);
	    impedance_control_x_.setDeadZone(dead_zone_[0]);
	    aic_control_x_.setImpedanceFilterParameters(M_[0], B_[0], K_[0]);
	    aic_control_x_.setAdaptiveParameters(kp1_[0], kp2_[0], wp_[0], wd_[0]);

	    impedance_control_y_.setImpedanceFilterMass(M_[1]);
	    impedance_control_y_.setImpedanceFilterDamping(B_[1]);
	    impedance_control_y_.setImpedanceFilterStiffness(K_[1]);
	    impedance_control_y_.setDeadZone(dead_zone_[1]);
	    aic_control_y_.setImpedanceFilterParameters(M_[1], B_[1], K_[1]);
	    aic_control_y_.setAdaptiveParameters(kp1_[1], kp2_[1], wp_[1], wd_[1]);

	    impedance_control_z_.setImpedanceFilterMass(M_[2]);
	    impedance_control_z_.setImpedanceFilterDamping(B_[2]);
	    impedance_control_z_.setImpedanceFilterStiffness(K_[2]);
	    impedance_control_z_.setDeadZone(dead_zone_[2]);
	    aic_control_z_.setImpedanceFilterParameters(M_[2], B_[2], K_[2]);
	    aic_control_z_.setAdaptiveParameters(kp1_[2], kp2_[2], wp_[2], wd_[2]);

	    // Call the dynamic reconfigure callback to initialize dynamic reconfigure server with these parameters
	    impedance_control::ImpedanceControlConfig dyn_config;
	    dyn_config.kp1_x = kp1_[0];
	    dyn_config.kp2_x = kp2_[0];
	    dyn_config.wp_x = wp_[0];
	    dyn_config.wd_x = wd_[0];
	    dyn_config.kp1_y = kp1_[1];
	    dyn_config.kp2_y = kp2_[1];
	    dyn_config.wp_y = wp_[1];
	    dyn_config.wd_y = wd_[1];
	    dyn_config.kp1_z = kp1_[2];
	    dyn_config.kp2_z = kp2_[2];
	    dyn_config.wp_z = wp_[2];
	    dyn_config.wd_z = wd_[2];

	    // dyn_config.M_x = M_[0];
	    // dyn_config.B_x = B_[0];
	    // dyn_config.K_x = K_[0];
	    // dyn_config.dead_zone_x = dead_zone_[0];

	    // dyn_config.M_y = M_[1];
		// dyn_config.B_y = B_[1];
	    // dyn_config.K_y = K_[1];
	    // dyn_config.dead_zone_y = dead_zone_[1];

	    // dyn_config.M_z = M_[2];
	    // dyn_config.B_z = B_[2];
	    // dyn_config.K_z = K_[2];
	    // dyn_config.dead_zone_z = dead_zone_[2];

	    // Invoke the callback to sync dynamic reconfigure with the loaded YAML values
	    reconfigureCb(dyn_config, 0);
	}


	// flag of measurements
	bool isReady(void)
	{
        // ROS_INFO("force_msg_received_: %s", force_msg_received_ ? "true" : "false");
        // ROS_INFO("pose_meas_received_: %s", pose_meas_received_ ? "true" : "false");

		return force_msg_received_ && pose_meas_received_;
	};

	// state machine of aic
	bool startImpedanceControlCb(std_srvs::SetBool::Request  &req, std_srvs::SetBool::Response &res)
	{
	    bool service_flag = false;
	    double initial_values[3];

		// ROS_INFO("required data is %s", req.data ? "true" : "false");
        // ROS_INFO("isReady %s", isReady() ? "true" : "false");
		// ROS_INFO("trajectory is recived: %s", (trajectory_msg_received_ ) ? "true" : "false");

	    if (req.data && isReady() && !impedance_start_flag_ && trajectory_msg_received_ && carrot_in_hold_mode_)
	    {
	        initial_values[0] = pose_meas_.pose.position.x; // tu ide carrot pozicija i orjentacije i sve
	        initial_values[1] = pose_meas_.pose.position.y;
	        initial_values[2] = pose_meas_.pose.position.z;

			// ROS_INFO("new pose x is = %.2f", pose_ref_.pose.position.x);
			// ROS_INFO("new pose y is = %.2f", pose_ref_.pose.position.y);
			// ROS_INFO("new pose x is = %.2f", pose_ref_.pose.position.z);

	        pose_ref_.pose.position.x = initial_values[0];
	        pose_ref_.pose.position.y = initial_values[1];
	        pose_ref_.pose.position.z = initial_values[2];

			double yaw = getYaw(
					pose_meas_.pose.orientation.x,
					pose_meas_.pose.orientation.y,
					pose_meas_.pose.orientation.z,
					pose_meas_.pose.orientation.w);
	
			pose_ref_.pose.orientation = calculateQuaternion(yaw);

	        vel_ref_.linear.x = 0;
	        vel_ref_.linear.y = 0;
	        vel_ref_.linear.z = 0;
	        vel_ref_.angular.x = 0;
	        vel_ref_.angular.y = 0;
	        vel_ref_.angular.z = 0;

	        acc_ref_.linear.x = 0;
	        acc_ref_.linear.y = 0;
	        acc_ref_.linear.z = 0;
	        acc_ref_.angular.x = 0;
	        acc_ref_.angular.y = 0;
	        acc_ref_.angular.z = 0;

	        ROS_INFO("Starting impedance control.");
	        initializeImpedanceFilterTransferFunction();
	        setImpedanceFilterInitialValue(initial_values);

			initializeAdaptationLaws();
			setAdaptiveEnvironmentStiffnessInitialValue();

			// only set for true
	        impedance_start_flag_ = true;
	        service_flag = true;
	    }
	    else if (!req.data)
	    {
	        ROS_INFO("Stoping impedance control.");
	        impedance_start_flag_ = false;
	        force_msg_received_ = false;
	        pose_meas_received_ = false;
	        service_flag = true;
	    }

	    res.success = service_flag;

	    return true;
	};

	// periferals
    ros::Time getTime() {
        ros::Time time;

		if (simulation_flag_)
			time = clock_.clock;
		else 
			time = ros::Time::now();

		return time;
	};

	bool isStarted(void) {
		return impedance_start_flag_;
	};

	// main filter logic 
	void impedanceFilter() {
		double xd[3], yd[3], zd[3];

		xd[0] = pose_ref_.pose.position.x;
        xd[1] = vel_ref_.linear.x;
        xd[2] = acc_ref_.linear.x;

        yd[0] = pose_ref_.pose.position.y;
        yd[1] = vel_ref_.linear.y;
        yd[2] = acc_ref_.linear.y;

        zd[0] = pose_ref_.pose.position.z;
        zd[1] = vel_ref_.linear.z;
        zd[2] = acc_ref_.linear.z;

        qxc_[0] = pose_ref_.pose.orientation.x;
		qxc_[1] = 0.0;
		qxc_[2] = 0.0;

		qyc_[0] = pose_ref_.pose.orientation.y;
		qyc_[1] = 0.0;
		qyc_[2] = 0.0;

		qzc_[0] = pose_ref_.pose.orientation.z;
		qzc_[1] = 0.0;
		qzc_[2] = 0.0;

		qwc_[0] = pose_ref_.pose.orientation.w;
		qwc_[1] = 0.0;
		qwc_[2] = 0.0;

		// xd = Xe in diagram
		xr_ = aic_control_x_.compute(force_meas_.wrench.force.x, force_torque_ref_.wrench.force.x, xd);
        xKp_ = aic_control_x_.getAdaptiveEnvironmentStiffnessGainKp();
		// xr_ = Xr in diagram -> output of adaptive impedance to impedance filter
        xq_ = aic_control_x_.getQ();

		// inirial Xe (Xd) from MPC is passde to filter and it is creating new corrected trajectory without adaptation
        xc_ = impedance_control_x_.impedanceFilter(force_meas_.wrench.force.x, force_torque_ref_.wrench.force.x, xd);

        yr_ = aic_control_y_.compute(force_meas_.wrench.force.y, force_torque_ref_.wrench.force.y, yd);
        yKp_ = aic_control_y_.getAdaptiveEnvironmentStiffnessGainKp();
        yq_ = aic_control_y_.getQ();
        yc_ = impedance_control_y_.impedanceFilter(force_meas_.wrench.force.y, force_torque_ref_.wrench.force.y, yd);

        zr_ = aic_control_z_.compute(force_meas_.wrench.force.z, force_torque_ref_.wrench.force.z, zd);
        zKp_ = aic_control_z_.getAdaptiveEnvironmentStiffnessGainKp();
        zq_ = aic_control_z_.getQ();
        zc_ = impedance_control_z_.impedanceFilter(force_meas_.wrench.force.z, force_torque_ref_.wrench.force.z, zd);

	};

	// getters and setters, to be changed with struct

	double *getXc() {
		return xc_;
	};

    double *getYc() {
        return yc_;
    };

    double *getZc() {
        return zc_;
    };

	double *getQXc() {
		return qxc_;
	};

	double *getQYc() {
		return qyc_;
	};

	double *getQZc() {
		return qzc_;
	};

	double *getQWc() {
		return qwc_;
	};

    double *getXr() {
        return xr_;
    };

    double *getYr() {
        return yr_;
    };

    double *getZr() {
        return zr_;
    };

    double *getXKp() {
        return xKp_;
    };

    double *getYKp() {
        return yKp_;
    };

    double *getZKp() {
        return zKp_;
    };

    double getXq() {
        return xq_;
    };

    double getYq() {
        return yq_;
    };

    double getZq() {
        return zq_;
    };
};

int main(int argc, char **argv) {

	// init memory
	int rate, impedance_type[3];
	std::string impedance_control_config_file;
	bool simulation_flag;
	double time, time_old, dt;
	double *xc, *yc, *zc, *xr, *yr, *zr, *xKp, *yKp, *zKp;
	double *qxc, *qyc, *qzc, *qwc;
	double xq, yq, zq;

	// variables for tracking impedance start
	double start_time = 0.0;  // Variable to track the start time
	bool is_impedance_started = false;  // Flag to track if impedance has been started

	// msgs for control and output preparation
	geometry_msgs::PoseStamped commanded_position_msg;
	std_msgs::Float64MultiArray state_msg;
	trajectory_msgs::MultiDOFJointTrajectoryPoint commanded_traj_point;
	commanded_traj_point.transforms = std::vector<geometry_msgs::Transform>(1);
	commanded_traj_point.velocities = std::vector<geometry_msgs::Twist>(1);
	commanded_traj_point.accelerations = std::vector<geometry_msgs::Twist>(1);

	state_msg.data.resize(31);

	ros::init(argc, argv, "impedance_control_node");
	ros::NodeHandle n, private_node_handle_("~");

	std::string path = ros::package::getPath("impedance_control");

	private_node_handle_.param("rate", rate, int(100));
	private_node_handle_.param("impedance_control_params_file", impedance_control_config_file, std::string("/config/impedance_control_params.yaml"));
	private_node_handle_.param("simulation", simulation_flag, bool(true));

	ImpedanceControlNode impedance_control(rate, simulation_flag);
	impedance_control.loadImpedanceControlParameters(path+impedance_control_config_file);

	ros::Subscriber clock_ros_sub = n.subscribe("/clock", 1, &ImpedanceControlNode::clockCb, &impedance_control);

	// force measurements for feedback
	ros::Subscriber force_ros_sub = n.subscribe("force_sensor/force_torque_filtered", 1, &ImpedanceControlNode::forceMeasurementCb, &impedance_control);
 
	//subscribe to state in order to trigger after takeoff
	ros::Subscriber carrot_State = n.subscribe("carrot/status", 1, &ImpedanceControlNode::carrotStatusCb, &impedance_control);

	// remove -> comes from trajectorys next point
	//ros::Subscriber pose_ref_ros_sub = n.subscribe("tracker/input_pose", 1, &ImpedanceControlNode::poseRefCb, &impedance_control);

	//  to be removede comes from previous trajectroy
	// ros::Subscriber trajectory_ref_ros_sub = n.subscribe("position_hold/trajectory", 1, &ImpedanceControlNode::trajectoryRefCb, &impedance_control);

	// Xe -> desired trajectory
	ros::Subscriber trajectory_point_ref_ros_sub = n.subscribe("position_hold/trajectory", 1, &ImpedanceControlNode::trajectoryPointRefCb, &impedance_control);

	// sub to force reference 
	ros::Subscriber force_torque_ref_ros_sub = n.subscribe("force_reference", 1, &ImpedanceControlNode::forceTorqueRefCb, &impedance_control);

	//sub to odometry -> can be removed and obtained from trajectory
	// ros::Subscriber pose_meas_odometry_sub = n.subscribe("odometry", 1, &ImpedanceControlNode::poseMeasOdomCb, &impedance_control);
	ros::Subscriber pose_meas_odometry_sub = n.subscribe("carrot/trajectory", 1, &ImpedanceControlNode::carrotMeasOdomCb, &impedance_control);

    // got from odeomtry before
	// ros::Subscriber pose_meas_posestamped_sub = n.subscribe("impedance_control/pose_stamped_meas_input", 1, &ImpedanceControlNode::poseMeasPoseStampedCb, &impedance_control);

	// no need to modify wanted pose -> MPC in loop
	ros::Publisher pose_stamped_commanded_pub_ = n.advertise<geometry_msgs::PoseStamped>("impedance_control/pose_stamped_output", 1);

	// trajectory for carrot -> Xc
	ros::Publisher trajectory_point_command_pub_ = n.advertise<trajectory_msgs::MultiDOFJointTrajectoryPoint>("position_hold/trajectory_aic_corrected", 1);

	// trajectory Xc already published
	// ros::Publisher pose_commanded_pub_ = n.advertise<geometry_msgs::Pose>("impedance_control/pose_output", 1);

	// publish of states, arbitrarry 
	ros::Publisher state_pub_ = n.advertise<std_msgs::Float64MultiArray>("impedance_control/state", 1);

	// starting flag to be published
	ros::ServiceServer start_impedance_control_ros_srv = n.advertiseService("impedance_control/start", &ImpedanceControlNode::startImpedanceControlCb, &impedance_control);
	
	// Create a service client for the SetBool service
	ros::ServiceClient client = n.serviceClient<std_srvs::SetBool>("impedance_control/start");

	dynamic_reconfigure::Server<impedance_control::ImpedanceControlConfig> server;
    dynamic_reconfigure::Server<impedance_control::ImpedanceControlConfig>::CallbackType reconfigure;
    
    reconfigure = boost::bind(&ImpedanceControlNode::reconfigureCb, &impedance_control, _1, _2);
    server.setCallback(reconfigure);

	ros::Rate loop_rate(rate);

	// Petry state machine

	while (ros::Time::now().toSec() == 0 && ros::ok()) {
        ROS_INFO("Waiting for clock server to start 1");
        ros::Duration(0.5).sleep();
    }
    ROS_INFO("Received first clock message");

    while (!impedance_control.isReady() && ros::ok()) {
        ros::spinOnce();

        ROS_INFO("Waiting for the first measurement.");
        ros::Duration(0.5).sleep();
    }
    ROS_INFO("Waiting impedance control to start...");

    time_old = impedance_control.getTime().toSec();

	// ros loop for control -> main calls
    while (ros::ok()) {
        ros::spinOnce();

    	time = impedance_control.getTime().toSec();

        dt = time - time_old;
        time_old = time;

		// dt is 0.01s -> 100Hz update rate from trajectory
		//ROS_INFO("dt is: %f, and time old is: %f", dt, time_old);

		// Create the service request
    	std_srvs::SetBool srv;
    	srv.request.data = true; // Set to true to start impedance control

    	// Call the service
    	if (impedance_control.startImpedanceControlCb(srv.request, srv.response)) {
    	    if (srv.response.success) {
    	        ROS_INFO("Impedance control started successfully.");
    	    }
    	} else {
    	    ROS_ERROR("Failed to call the function.");
    	}

		// if time has changed in between timestamps -> it enters this loop
    	if (dt > 0.0) {

			//check if ready
			//ROS_INFO("imped has started: %s", impedance_control.isStarted()? "true" : "false");

        	if (impedance_control.isStarted()) { 
				
				// create filter sturct
                impedance_control.impedanceFilter();

				// init filter params
        		xc = impedance_control.getXc();
                yc = impedance_control.getYc();
                zc = impedance_control.getZc();
				qxc = impedance_control.getQXc();
				qyc = impedance_control.getQYc();
				qzc = impedance_control.getQZc();
				qwc = impedance_control.getQWc();
                xr = impedance_control.getXr();
                yr = impedance_control.getYr();
                zr = impedance_control.getZr();
                xKp = impedance_control.getXKp();
                yKp = impedance_control.getYKp();
                zKp = impedance_control.getZKp();
                xq = impedance_control.getXq();
                yq = impedance_control.getYq();
                zq = impedance_control.getZq();

				// calculate next pose to mitigate error -> can be removed
        		// commanded_position_msg.header.stamp = impedance_control.getTime();
                // commanded_position_msg.pose.position.x = xc[0];
                // commanded_position_msg.pose.position.y = yc[0];
                // commanded_position_msg.pose.position.z = zc[0];
                // commanded_position_msg.pose.orientation.x = qxc[0];
                // commanded_position_msg.pose.orientation.y = qyc[0];
                // commanded_position_msg.pose.orientation.z = qzc[0];
                // commanded_position_msg.pose.orientation.w = qwc[0];
                // pose_stamped_commanded_pub_.publish(commanded_position_msg);
                // pose_commanded_pub_.publish(commanded_position_msg.pose);

				// Xc of interest
				commanded_traj_point.transforms[0].translation.x = xc[0];
				commanded_traj_point.transforms[0].translation.y = yc[0];
				commanded_traj_point.transforms[0].translation.z = zc[0];
				commanded_traj_point.velocities[0].linear.x = xc[1];
				commanded_traj_point.velocities[0].linear.y = yc[1];
				commanded_traj_point.velocities[0].linear.z = zc[1];
				commanded_traj_point.accelerations[0].linear.x = xc[2];
				commanded_traj_point.accelerations[0].linear.y = yc[2];
				commanded_traj_point.accelerations[0].linear.z = zc[2];
				// ROS_INFO("accel: \n x: %.2f\n y: %.2f\n z: %.2f", xc[2], yc[2], zc[2]);
				commanded_traj_point.transforms[0].rotation.x = qxc[0];
				commanded_traj_point.transforms[0].rotation.y = qyc[0];
				commanded_traj_point.transforms[0].rotation.z = qzc[0];
				commanded_traj_point.transforms[0].rotation.w = qwc[0];

				// trajectory to be published
				trajectory_point_command_pub_.publish(commanded_traj_point);
				
				// go trough states of impedance for update
                int j = 0;
                for (int i = 0; i < 3; i++) {

					// output of adaptive part entering filter
					// each is array of 3 [pos, vel, accel]
                	state_msg.data[j++] = xr[i];

					// ROS_INFO("Xr = %.2f", xr[i]);
                	state_msg.data[j++] = yr[i];
                	state_msg.data[j++] = zr[i];
					// output of filter for position controller
					// each is array of 3 [pos, vel, accel]
                	state_msg.data[j++] = xc[i];
					// ROS_INFO("Xc = %.2f", xc[i]);
                	state_msg.data[j++] = yc[i];
                	state_msg.data[j++] = zc[i];
					// params of gains
                	state_msg.data[j++] = xKp[i];
                    state_msg.data[j++] = yKp[i];
                    state_msg.data[j++] = zKp[i];
                }
                state_msg.data[j++] = xq;
                state_msg.data[j++] = yq;
                state_msg.data[j++] = zq;
				state_msg.data[j++] = time;
                state_pub_.publish(state_msg);
        	}
        }

		// wait for next iteration
        loop_rate.sleep();
    }

	return 0;
}
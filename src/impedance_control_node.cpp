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
#include <impedance_control/ImpedanceControlConfig.h>

#include "yaml-cpp/yaml.h"

class ImpedanceControlNode {
private:
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

	bool simulation_flag_, force_msg_received_, reconfigure_start_;
	bool pose_meas_received_, impedance_start_flag_;
	float xc_[3], vc_[3], ac_[3], fe_[3];
	float xr_[3], vr_[3], ar_[3], Ke_[3];
	int rate_;

	std::vector<double> ke1_, ke2_, kp_;

	rosgraph_msgs::Clock clock_, clock_old_;
	geometry_msgs::WrenchStamped force_meas_, force_torque_ref_;
	geometry_msgs::PoseStamped pose_ref_, pose_meas_;
	geometry_msgs::Twist vel_ref_, acc_ref_;

	ImpedanceControl impedance_control_z_, impedance_control_y_, impedance_control_x_;
	aic aic_control_z_; 
public:
	ImpedanceControlNode(int rate, bool simulation):
		rate_(rate),
		simulation_flag_(simulation),
		force_msg_received_(false),
		pose_meas_received_(false),
		impedance_start_flag_(false),
		impedance_control_x_(rate),
		impedance_control_y_(rate),
		impedance_control_z_(rate),
		reconfigure_start_(false) {

		for (int i = 0; i < 3; i++) {
			xc_[i] = 0;
			vc_[i] = 0;
			ac_[i] = 0;
			fe_[i] = 0;

			Ke_[i] = 0;
			xr_[i] = 0;
			vr_[i] = 0;
			ar_[i] = 0;
		}
	};

	void clockCb(const rosgraph_msgs::Clock &msg) {
	    clock_ = msg;
	};

	void forceMeasurementCb(const geometry_msgs::WrenchStamped &msg) {
		force_msg_received_ = true;
		force_meas_ = msg;
	};

	void poseRefCb(const geometry_msgs::PoseStamped &msg) {
	    pose_ref_ = msg;
	};

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
	}

	void forceTorqueRefCb(const geometry_msgs::WrenchStamped &msg) {
    	force_torque_ref_ = msg;
	};

	void poseMeasOdomCb(const nav_msgs::Odometry &msg) {
		pose_meas_received_ = true;
		pose_meas_.header = msg.header;
		pose_meas_.pose = msg.pose.pose;
	};

	void reconfigureCb(impedance_control::ImpedanceControlConfig &config, uint32_t level) {
		float ke[3];

		if (!reconfigure_start_) {
			config.ke1_z = ke1_[2];
			config.ke2_z = ke2_[2];
			config.kp_z = kp_[2];
			reconfigure_start_ = true;
		}
		else {
			ke1_[2] = config.ke1_z;
			ke2_[2] = config.ke2_z;
			kp_[2] = config.kp_z;

			ke[0] = ke1_[2];
    		ke[1] = ke2_[2];
    		ke[2] = kp_[2];
			aic_control_z_.initializeAdaptationLaws(ke, (float)(1.0/rate_));
		}	
	};

	void loadImpedanceControlParameters(std::string file) {
		YAML::Node config = YAML::LoadFile(file);
		std::vector<double> B, K, M, dead_zone, Ke0;
		std::vector<int> impedance_type;
		float ke[3];

		M = config["IMPEDANCE_FILTER"]["M"].as<std::vector<double> >();
    	B = config["IMPEDANCE_FILTER"]["B"].as<std::vector<double> >();
    	K = config["IMPEDANCE_FILTER"]["K"].as<std::vector<double> >();
    	dead_zone = config["IMPEDANCE_FILTER"]["dead_zone"].as<std::vector<double> >();
    	impedance_type = config["IMPEDANCE_FILTER"]["impedance_type"].as<std::vector<int> >();
    	ke1_ = config["AIC"]["INTEGRAL_ADAPTATION_GAINS"]["ke1"].as<std::vector<double> >();
    	ke2_ = config["AIC"]["PROPORTIONAL_ADAPTATION_GAINS"]["ke2"].as<std::vector<double> >();
    	Ke0 = config["AIC"]["INITIAL_GAINS"]["Ke"].as<std::vector<double> >();
    	kp_ = config["AIC"]["PROPORTIONAL_ERROR_GAIN"]["kp"].as<std::vector<double> >();
    	
    	impedance_control_x_.setImpedanceFilterMass(M[0]);
    	impedance_control_x_.setImpedanceFilterDamping(B[0]);
    	impedance_control_x_.setImpedanceFilterStiffness(K[0]);
    	impedance_control_x_.setTargetImpedanceType(impedance_type[0]);
    	impedance_control_x_.setDeadZone(dead_zone[0]);

    	impedance_control_y_.setImpedanceFilterMass(M[1]);
    	impedance_control_y_.setImpedanceFilterDamping(B[1]);
    	impedance_control_y_.setImpedanceFilterStiffness(K[1]);
    	impedance_control_y_.setTargetImpedanceType(impedance_type[1]);
    	impedance_control_y_.setDeadZone(dead_zone[1]);

    	impedance_control_z_.setImpedanceFilterMass(M[2]);
    	impedance_control_z_.setImpedanceFilterDamping(B[2]);
    	impedance_control_z_.setImpedanceFilterStiffness(K[2]);
    	impedance_control_z_.setTargetImpedanceType(impedance_type[2]);
    	impedance_control_z_.setDeadZone(dead_zone[2]);
    	aic_control_z_.setAdaptiveParameterInitialValues(Ke0[2]);
    	ke[0] = ke1_[2];
    	ke[1] = ke2_[2];
    	ke[3] = kp_[2];
    	aic_control_z_.initializeAdaptationLaws(ke, (float)(1.0/rate_));
	};

	bool isReady(void)
	{
		return force_msg_received_ && pose_meas_received_;
	};

	bool startImpedanceControlCb(std_srvs::SetBool::Request  &req, std_srvs::SetBool::Response &res)
	{
	    bool service_flag = false;
	    double initial_values[3];

	    if (req.data && isReady() && !impedance_start_flag_)
	    {
	        initial_values[0] = pose_meas_.pose.position.x;
	        initial_values[1] = pose_meas_.pose.position.y;
	        initial_values[2] = pose_meas_.pose.position.z;

	        pose_ref_.pose.position.x = initial_values[0];
	        pose_ref_.pose.position.y = initial_values[1];
	        pose_ref_.pose.position.z = initial_values[2];
	        pose_ref_.pose.orientation.x = pose_meas_.pose.orientation.x;
	        pose_ref_.pose.orientation.y = pose_meas_.pose.orientation.y;
	        pose_ref_.pose.orientation.z = pose_meas_.pose.orientation.z;
	        pose_ref_.pose.orientation.w = pose_meas_.pose.orientation.w;

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

	double getTime() {
		double time;

		if (simulation_flag_)
			time = clock_.clock.toSec();
		else 
			time = ros::Time::now().toSec();

		return time;
	};

	bool isStarted(void) {
		return impedance_start_flag_;
	};

	void impedanceFilter() {
		float X[3];

		fe_[0] = 0.0;//force_torque_ref_.wrench.force.x - force_meas_.wrench.force.x;
		impedance_control_x_.impedanceFilter(fe_[0], pose_ref_.pose.position.x, vel_ref_.linear.x, acc_ref_.linear.x, X);
		xc_[0] = X[0];

		fe_[1] = 0.0;//force_torque_ref_.wrench.force.y - force_meas_.wrench.force.y;
		impedance_control_y_.impedanceFilter(fe_[1], pose_ref_.pose.position.y, vel_ref_.linear.y, acc_ref_.linear.z, X);
		xc_[1] = X[0]; 

		fe_[2] = -(force_torque_ref_.wrench.force.z - force_meas_.wrench.force.z);
		xr_[2] = aic_control_z_.compute(fe_[2], -force_torque_ref_.wrench.force.z, pose_ref_.pose.position.z);
		Ke_[2] = aic_control_z_.getAdaptiveEnvironmentStiffnessGainKe();
		impedance_control_z_.impedanceFilter(fe_[2], xr_[2], 0, 0, X);
		xc_[2] = X[0];
		vc_[2] = X[1];
		ac_[2] = X[2];
	};

	float *getXc() {
		return xc_;
	};

	float *getVc() {
		return vc_;
	};

	float *getAc() {
		return ac_;
	};

	float *getXr() {
		return xr_;
	};

	float *getVr() {
		return vr_;
	};

	float *getAr() {
		return ar_;
	};

	float *getKe() {
		return Ke_;
	};
};

int main(int argc, char **argv) {
	int rate, impedance_type[3];
	std::string impedance_control_config_file;
	bool simulation_flag;
	double time, time_old, dt;
	float *xc, dead_zone;

	geometry_msgs::PoseStamped commanded_position_msg;

	ros::init(argc, argv, "impedance_control_node");
	ros::NodeHandle n, private_node_handle_("~");

	std::string path = ros::package::getPath("impedance_control");

	private_node_handle_.param("rate", rate, int(100));
	private_node_handle_.param("impedance_control_params_file", impedance_control_config_file, std::string("/config/impedance_control_params.yaml"));
	private_node_handle_.param("simulation", simulation_flag, bool(true));

	ImpedanceControlNode impedance_control(rate, simulation_flag);
	impedance_control.loadImpedanceControlParameters(path+impedance_control_config_file);

	ros::Subscriber clock_ros_sub = n.subscribe("/clock", 1, &ImpedanceControlNode::clockCb, &impedance_control);
	ros::Subscriber force_ros_sub = n.subscribe("/force_sensor/filtered_ft_sensor", 1, &ImpedanceControlNode::forceMeasurementCb, &impedance_control);
	ros::Subscriber pose_ref_ros_sub = n.subscribe("impedance_control/pose_ref", 1, &ImpedanceControlNode::poseRefCb, &impedance_control);
	ros::Subscriber trajectory_ref_ros_sub = n.subscribe("impedance_control/trajectory_ref", 1, &ImpedanceControlNode::trajectoryRefCb, &impedance_control);
	ros::Subscriber force_torque_ref_ros_sub = n.subscribe("impedance_control/force_torque_ref", 1, &ImpedanceControlNode::forceTorqueRefCb, &impedance_control);
	ros::Subscriber pose_meas_odometry_sub = n.subscribe("odometry", 1, &ImpedanceControlNode::poseMeasOdomCb, &impedance_control);

	ros::Publisher pose_commanded_pub_ = n.advertise<geometry_msgs::PoseStamped>("dual_arm_manipulator/set_point", 1);

	ros::ServiceServer start_impedance_control_ros_srv = n.advertiseService("impedance_control/start", &ImpedanceControlNode::startImpedanceControlCb, &impedance_control);

	dynamic_reconfigure::Server<impedance_control::ImpedanceControlConfig> server;
    dynamic_reconfigure::Server<impedance_control::ImpedanceControlConfig>::CallbackType reconfigure;
    
    reconfigure = boost::bind(&ImpedanceControlNode::reconfigureCb, &impedance_control, _1, _2);
    server.setCallback(reconfigure);

	ros::Rate loop_rate(rate);

	while (ros::Time::now().toSec() == 0 && ros::ok()) {
        ROS_INFO("Waiting for clock server to start");
    }
    ROS_INFO("Received first clock message");

    while (!impedance_control.isReady() && ros::ok()) {
        ros::spinOnce();

        ROS_INFO("Waiting for the first measurement.");
        ros::Duration(0.5).sleep();
    }
    ROS_INFO("Waiting impedance control to start...");

    time_old = impedance_control.getTime();

    while (ros::ok()) {
        ros::spinOnce();

    	time = impedance_control.getTime();

        dt = time - time_old;
        time_old = time;

        if (dt > 0.0) {
        	if (impedance_control.isStarted()) {
        		impedance_control.impedanceFilter();
        		xc = impedance_control.getXc();

        		commanded_position_msg.header.stamp = ros::Time::now();
                commanded_position_msg.pose.position.x = xc[0];
                commanded_position_msg.pose.position.y = xc[1];
                commanded_position_msg.pose.position.z = xc[2];
                commanded_position_msg.pose.orientation.x = impedance_control.getXr()[2];
                commanded_position_msg.pose.orientation.y = impedance_control.getKe()[2];
                commanded_position_msg.pose.orientation.z = impedance_control.getVc()[2];
                commanded_position_msg.pose.orientation.w = impedance_control.getAc()[2];
                pose_commanded_pub_.publish(commanded_position_msg);
        	}
        }

        loop_rate.sleep();
    }

	return 0;
}
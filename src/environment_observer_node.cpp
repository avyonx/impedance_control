#include "ros/ros.h"
#include <impedance_control/environment_observer.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <rosgraph_msgs/Clock.h>

class EnvironmentObserverNode {
private:
	bool simulation_flag_, force_msg_received_;
	bool pose_meas_received_, pose_ref_received_;

	geometry_msgs::WrenchStamped force_meas_;
	geometry_msgs::Point position_meas_, position_ref_;
	rosgraph_msgs::Clock clock_, clock_old_;

	EnvironmentObserver observer_;
public:
	EnvironmentObserverNode(bool simulation_flag) :
		simulation_flag_(simulation_flag),
		force_msg_received_(false),
		pose_meas_received_(false),
		pose_ref_received_(false)
	{
		observer_.initCovValues();
	};

	void forceMeasCb(const geometry_msgs::WrenchStamped &msg)
	{
		force_msg_received_ = true;
		force_meas_ = msg;
	};

	void poseMeasOdomCb(const nav_msgs::Odometry &msg)
	{
		pose_meas_received_ = true;
		position_meas_.x = msg.pose.pose.position.x;
		position_meas_.y = msg.pose.pose.position.y;
		position_meas_.z = msg.pose.pose.position.z;
	};

	void poseRefPoseStampedCb(const geometry_msgs::PoseStamped &msg)
	{
		pose_ref_received_ = true;
		position_ref_.x = msg.pose.position.x;
		position_ref_.y = msg.pose.position.y;
		position_ref_.z = msg.pose.position.z;
	};

	void clockCb(const rosgraph_msgs::Clock &msg)
	{
	    clock_ = msg;
	};

	bool isReady(void)
	{
		return force_msg_received_ && pose_meas_received_ && pose_ref_received_;
	};

	void initStateVector()
	{
		double dxe0, Ke0, De0, f0;

		dxe0 = position_meas_.z - position_ref_.z;
		Ke0 = 500.0;
		De0 = 1.0;
		f0 = force_meas_.wrench.force.z;

		clock_old_ = clock_;

		observer_.initStateVector(dxe0, Ke0, De0, f0);
	};

	void modelUpdate(double dt) {
		observer_.modelUpdate(dt);
	};

	void measureUpdate(void) {
		double dxe, fe;

		dxe = position_meas_.z - position_ref_.z;
		fe = force_meas_.wrench.force.z;

		observer_.measureUpdate(dxe, fe);
	};

	double getDt(void) {
		double dt = 0;

		if (simulation_flag_) {
			dt = (clock_.clock.toSec() - clock_old_.clock.toSec());
			clock_old_ = clock_;
		}
		else {

		}

		return dt;
	};

	double getRealDxe() {
		return position_meas_.z - position_ref_.z;
	};

	double getFe(void) {
		return observer_.getFe();
	};

	double getKe(void) {
		return observer_.getKe();
	};

	double getDXe(void) {
		return observer_.getDXe();
	};

	double getDe(void) {
		return observer_.getDe();
	};

};

int main(int argc, char **argv) {
	ros::init(argc, argv, "environment_observer_node");
	ros::NodeHandle n, private_node_handle_("~");

	int rate;
	double dt;
	bool simulation_flag;
	geometry_msgs::WrenchStamped force_estimated_msg;
	geometry_msgs::PointStamped stiffness_estimated_msg;
	geometry_msgs::PointStamped damping_estimated_msg;
	geometry_msgs::PointStamped dx_estimated_msg;

	private_node_handle_.param("rate", rate, int(100));
	private_node_handle_.param("simulation", simulation_flag, bool(true));

	EnvironmentObserverNode observer(simulation_flag);

	ros::Subscriber force_meas_sub = n.subscribe("/force_sensor/filtered_ft_sensor", 1, &EnvironmentObserverNode::forceMeasCb, &observer);
	ros::Subscriber pose_meas_odometry_sub = n.subscribe("/mmuav/odometry", 1, &EnvironmentObserverNode::poseMeasOdomCb, &observer);
	ros::Subscriber pose_ref_pose_stamped_sub = n.subscribe("/mmuav/dual_arm_manipulator/current_reference", 1, &EnvironmentObserverNode::poseRefPoseStampedCb, &observer);
	ros::Subscriber clock_ros_sub_ = n.subscribe("/clock", 1, &EnvironmentObserverNode::clockCb, &observer);

	ros::Publisher force_estimated_pub = n.advertise<geometry_msgs::WrenchStamped>("/environment_observer/force_estimated", 1);
	ros::Publisher stiffness_estimated_pub = n.advertise<geometry_msgs::PointStamped>("/environment_observer/stiffness_estimated", 1);
	ros::Publisher damping_estimated_pub = n.advertise<geometry_msgs::PointStamped>("/environment_observer/damping_estimated", 1);
	ros::Publisher dx_estimated_pub = n.advertise<geometry_msgs::PointStamped>("/environment_observer/dx_estimated", 1);
	
	ros::Rate loop_rate(rate);

	while (ros::Time::now().toSec() == 0 && ros::ok()) {
        ROS_INFO("Waiting for clock server to start 2");
        ros::Duration(0.5).sleep();
    }
    ROS_INFO("Received first clock message");

    while (!observer.isReady() && ros::ok()) {
        ros::spinOnce();

        ROS_INFO("Waiting for the first measurement.");
        ros::Duration(0.5).sleep();
    }
    ROS_INFO("Starting observer node!");

    observer.initStateVector();

	while (ros::ok()) {
		ros::spinOnce();

		dt = observer.getDt();

		if (dt > 0.0) {
			observer.modelUpdate(dt);
			observer.measureUpdate();

			force_estimated_msg.header.stamp = ros::Time::now();
			force_estimated_msg.wrench.force.z = observer.getFe();
			force_estimated_pub.publish(force_estimated_msg);

			stiffness_estimated_msg.header.stamp = ros::Time::now();
			stiffness_estimated_msg.point.z = observer.getKe();
			stiffness_estimated_pub.publish(stiffness_estimated_msg);

			damping_estimated_msg.header.stamp = ros::Time::now();
			damping_estimated_msg.point.z = observer.getDe();
			damping_estimated_pub.publish(damping_estimated_msg);

			dx_estimated_msg.header.stamp = ros::Time::now();
			dx_estimated_msg.point.z = observer.getDXe();
			dx_estimated_pub.publish(dx_estimated_msg);
		}

		loop_rate.sleep();
	}

	return 0;
}
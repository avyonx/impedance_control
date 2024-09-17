#include <ros/ros.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h> // For trajectory message
#include <geometry_msgs/WrenchStamped.h> // For ft_sensor data
#include <nav_msgs/Odometry.h> // For odometry data

#include <impedance_control/impedance_control.h>
#include <impedance_control/aic.h>

class PoleAICInspection {
public:
    PoleAICInspection(ros::NodeHandle& nh, ros::NodeHandle& nh_private) {
        // Advertise the corrected trajectory topic
        trajectory_pub_ = nh.advertise<trajectory_msgs::MultiDOFJointTrajectoryPoint>("position_hold/trajectory_aic_corrected", 1);

        // Subscribe to necessary topics
        trajectory_sub_ = nh.subscribe("position_hold/trajectory", 1, &PoleAICInspection::trajectoryCallback, this);
        ft_sensor_sub_ = nh.subscribe("force_sensor/sensor_readings", 1, &PoleAICInspection::ftSensorCallback, this);
        odometry_sub_ = nh.subscribe("odometry", 1, &PoleAICInspection::odometryCallback, this);
    }

private:
    // ROS Publishers and Subscribers
    ros::Publisher trajectory_pub_;
    ros::Subscriber trajectory_sub_;
    ros::Subscriber ft_sensor_sub_;
    ros::Subscriber odometry_sub_;

    // Variables to store received data
    trajectory_msgs::MultiDOFJointTrajectoryPoint last_trajectory_msg_; // Latest trajectory message
    geometry_msgs::WrenchStamped last_ft_sensor_msg_; // Latest ft sensor message
    nav_msgs::Odometry last_odometry_msg_; // Latest odometry message

    // Force threshold
    const double FORCE_THRESHOLD_ = 2.0;

    // Callback to handle the incoming trajectory
    void trajectoryCallback(const trajectory_msgs::MultiDOFJointTrajectoryPoint::ConstPtr& msg) {
        // Store the received trajectory message in the global variable
        last_trajectory_msg_ = *msg;

        // Check if the force in x direction is less than the threshold
        if (std::abs(last_ft_sensor_msg_.wrench.force.x) < FORCE_THRESHOLD_) {
            // If the force is smaller than the threshold, publish the trajectory message
            trajectory_pub_.publish(last_trajectory_msg_); // No modification to trajectory needed
        } else {
            // aic correction
        }
    }

    // Callback to handle the incoming force-torque sensor data
    void ftSensorCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg) {
        // Store the received ft sensor message in the global variable
        last_ft_sensor_msg_ = *msg;
    }

    // Callback to handle the incoming odometry data
    void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        // Store the received odometry message in the global variable
        last_odometry_msg_ = *msg;
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "pole_aic_inspection");

    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    // Create the class instance
    PoleAICInspection pole_aic_inspection(nh, nh_private);

    ros::spin(); // Spin to process callbacks

    return 0;
}

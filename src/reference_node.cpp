#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <gazebo_msgs/GetModelState.h>
#include <vector>
#include <cmath>

class ReferenceNode {
public:
    ReferenceNode(ros::NodeHandle& nh, ros::NodeHandle& nh_private) 
        : nh_(nh), nh_private_(nh_private), force_published_(false) {

        // Initialize publishers
        force_ref_pub_ = nh_.advertise<geometry_msgs::WrenchStamped>("force_reference", 1000);
        pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("tracker/input_pose", 1000);

        // Define the service client to get model state
        client_ = nh_.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
        
        // Initialize pose
        updateBoxPose();
        setPoseInFront();
    }

    void run() {
        ros::Rate rate(1.0); // Run at 1 Hz to update pose
        ros::Time start_time = ros::Time::now(); // Record start time

        while (ros::ok()) {
            // Update the pose of the box
            updateBoxPose();
            
            // Publish the new pose in front of the box
            publishPoseInFront();

             // Check if 75 seconds have passed
            if (!force_published_ && (ros::Time::now() - start_time).toSec() >= 55.0) {
                publishPoseInFront();

            }

            // Check if 75 seconds have passed
            if (!force_published_ && (ros::Time::now() - start_time).toSec() >= 85.0) {
                publishForceReference(1.5); // Publish 1.5 N force
                force_published_ = true; // Set flag to prevent republishing
            }

            // Sleep for the publishing rate
            rate.sleep();
        }
    }

private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    ros::Publisher force_ref_pub_;
    ros::Publisher pose_pub_;
    ros::ServiceClient client_;
    
    // Box pose
    geometry_msgs::Pose box_pose_;
    geometry_msgs::PoseStamped target_pose_;
    bool force_published_;

    void updateBoxPose() {
        gazebo_msgs::GetModelState srv;
        srv.request.model_name = "box_right";
        if (client_.call(srv)) {
            box_pose_ = srv.response.pose;
        } else {
            ROS_ERROR("Failed to call service get_model_state");
        }
    }

    void setPoseInFront() {
        // Calculate the new position front of the box
        double distance = 2.227;
        target_pose_.pose.position.x = box_pose_.position.x - distance;
        target_pose_.pose.position.y = 0;
        target_pose_.pose.position.z = 2; 

        target_pose_.pose.orientation.x = 0.0;
        target_pose_.pose.orientation.y = 0.0;
        target_pose_.pose.orientation.z = 0.0;
        target_pose_.pose.orientation.w = 1.0;

        ROS_INFO("Setting target position to: [%f, %f, %f]", 
                 target_pose_.pose.position.x, target_pose_.pose.position.y, target_pose_.pose.position.z);

        ROS_INFO("Setting target orientation to: [%f, %f, %f, %f]", 
                 target_pose_.pose.orientation.x, target_pose_.pose.orientation.y, target_pose_.pose.orientation.z, target_pose_.pose.orientation.w);
    }

    void publishPoseInFront() {
        target_pose_.header.stamp = ros::Time::now(); // Set the timestamp
        target_pose_.header.frame_id = "pole_tip";  
        pose_pub_.publish(target_pose_);
    }

    void publishForceReference(double force) {
        geometry_msgs::WrenchStamped wrench_msg;
        wrench_msg.header.stamp = ros::Time::now(); // Set the timestamp
        wrench_msg.header.frame_id = "pole_tip"; 

        wrench_msg.wrench.force.x = force; // Set the force
        wrench_msg.wrench.force.y = 0.0;
        wrench_msg.wrench.force.z = 0.0;

        ROS_INFO("Publishing force reference: [%f, 0.0, 0.0]", force);
        force_ref_pub_.publish(wrench_msg);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "reference_node");

    // Global and private node handles
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    // Create the ReferenceNode object
    ReferenceNode reference_node(nh, nh_private);

    // Run the node
    reference_node.run();

    return 0;
}

#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/PoseStamped.h>

class ForceReferenceNode {
public:
    ForceReferenceNode(ros::NodeHandle& nh, ros::NodeHandle& nh_private) 
        : nh_(nh), nh_private_(nh_private) {

        // Initialize publishers
        force_ref_pub_ = nh_.advertise<geometry_msgs::WrenchStamped>("force_reference", 1000);
        pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("tracker/input_pose", 1000);

        // Get the force reference values from parameters or default to zeros
        nh_private_.param("force_x", force_x_, 0.0);
        nh_private_.param("force_y", force_y_, 0.0);
        nh_private_.param("force_z", force_z_, 0.0);
        
        // Get the torque reference values from parameters or default to zeros
        nh_private_.param("torque_x", torque_x_, 0.0);
        nh_private_.param("torque_y", torque_y_, 0.0);
        nh_private_.param("torque_z", torque_z_, 0.0);

        // Define the positions for the square around the origin
        geometry_msgs::Point p1, p2, p3, p4;

        // Manually set the coordinates for each point in the square
        p1.x = 1.0; p1.y = 1.0; p1.z = 1.0;
        p2.x = 1.0; p2.y = -1.0; p2.z = 1.0;
        p3.x = -1.0; p3.y = -1.0; p3.z = 1.0;
        p4.x = -1.0; p4.y = 1.0; p4.z = 1.0;

        // Store the points in the positions vector
        positions_.push_back(p1);
        positions_.push_back(p2);
        positions_.push_back(p3);
        positions_.push_back(p4);

        current_position_idx_ = 0;
    }

    void run() {
        ros::Rate force_rate(1000); // 1000 Hz loop rate for force publishing
        ros::Rate pose_rate(1.0 / 15.0); // Publish pose every 15 seconds

        while (ros::ok()) {
            // Publish the wrench stamped message
            publishForceReference();

            // Every 15 seconds, publish the next position in the square
            publishNextPose();

            force_rate.sleep(); // Sleep for force publisher
            pose_rate.sleep(); // Sleep for pose publisher (15 seconds interval)
        }
    }

private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    ros::Publisher force_ref_pub_;
    ros::Publisher pose_pub_;

    // Force reference values
    double force_x_;
    double force_y_;
    double force_z_;

    // Torque reference values
    double torque_x_;
    double torque_y_;
    double torque_z_;

    // Position list for square movement
    std::vector<geometry_msgs::Point> positions_;
    size_t current_position_idx_;

    // Function to publish the force reference
    void publishForceReference() {
        geometry_msgs::WrenchStamped wrench_msg;
        wrench_msg.header.stamp = ros::Time::now(); // Set the timestamp
        wrench_msg.header.frame_id = "base_link"; // Set the frame ID (modify as needed)

        wrench_msg.wrench.force.x = force_x_;
        wrench_msg.wrench.force.y = force_y_;
        wrench_msg.wrench.force.z = force_z_;
        wrench_msg.wrench.torque.x = torque_x_;
        wrench_msg.wrench.torque.y = torque_y_;
        wrench_msg.wrench.torque.z = torque_z_;

        force_ref_pub_.publish(wrench_msg);
    }

    // Function to publish the next pose in the square sequence
    void publishNextPose() {
        geometry_msgs::PoseStamped pose_msg;
        pose_msg.header.stamp = ros::Time::now(); // Set the timestamp
        pose_msg.header.frame_id = "base_link";   // Set the frame ID (modify as needed)

        // Set the position
        pose_msg.pose.position = positions_[current_position_idx_];

        // Set the orientation (0,0,0,1 for a unit quaternion)
        pose_msg.pose.orientation.x = 0.0;
        pose_msg.pose.orientation.y = 0.0;
        pose_msg.pose.orientation.z = 0.0;
        pose_msg.pose.orientation.w = 1.0;

        // Publish the pose
        pose_pub_.publish(pose_msg);

        // Move to the next position, and loop back to the first one after completing the square
        current_position_idx_ = (current_position_idx_ + 1) % positions_.size();
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "force_reference_node");

    // Global and private node handles
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    // Create the ForceReferenceNode object
    ForceReferenceNode force_ref_node(nh, nh_private);

    // Run the node at 1000 Hz for force reference publishing, 15 seconds for pose publishing
    force_ref_node.run();

    return 0;
}

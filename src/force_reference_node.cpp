#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>

class ForceReferenceNode {
public:
    ForceReferenceNode(ros::NodeHandle& nh, ros::NodeHandle& nh_private) 
        : nh_(nh), nh_private_(nh_private) {

        // Initialize publisher
        force_ref_pub_ = nh_.advertise<geometry_msgs::WrenchStamped>("force_reference", 1000);

        // Get the force reference values from parameters or default to zeros
        nh_private_.param("force_x", force_x_, 0.0);
        nh_private_.param("force_y", force_y_, 0.0);
        nh_private_.param("force_z", force_z_, 0.0);
        
        // Get the torque reference values from parameters or default to zeros
        nh_private_.param("torque_x", torque_x_, 0.0);
        nh_private_.param("torque_y", torque_y_, 0.0);
        nh_private_.param("torque_z", torque_z_, 0.0);
    }

    void run() {
        ros::Rate rate(1000); // 1000 Hz loop rate

        while (ros::ok()) {
            // Prepare the wrench stamped message
            geometry_msgs::WrenchStamped wrench_msg;
            wrench_msg.header.stamp = ros::Time::now(); // Set the timestamp
            wrench_msg.header.frame_id = "base_link"; // Set the frame ID (modify as needed)

            wrench_msg.wrench.force.x = force_x_;
            wrench_msg.wrench.force.y = force_y_;
            wrench_msg.wrench.force.z = force_z_;
            wrench_msg.wrench.torque.x = torque_x_;
            wrench_msg.wrench.torque.y = torque_y_;
            wrench_msg.wrench.torque.z = torque_z_;

            // Publish the wrench stamped reference
            // force_ref_pub_.publish(wrench_msg);

            rate.sleep();
        }
    }

private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    ros::Publisher force_ref_pub_;

    // Force reference values
    double force_x_;
    double force_y_;
    double force_z_;

    // Torque reference values
    double torque_x_;
    double torque_y_;
    double torque_z_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "force_reference_node");

    // Global and private node handles
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    // Create the ForceReferenceNode object
    ForceReferenceNode force_ref_node(nh, nh_private);

    // Run the node at 1000 Hz
    force_ref_node.run();

    return 0;
}

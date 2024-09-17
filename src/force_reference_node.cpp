#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>

class ForceReferenceNode {
public:
    ForceReferenceNode(ros::NodeHandle& nh, ros::NodeHandle& nh_private) 
        : nh_(nh), nh_private_(nh_private) {

        // Initialize publisher
        force_ref_pub_ = nh_.advertise<geometry_msgs::Vector3>("force_reference", 1000);

        // Get the force reference values from parameters or default to zeros
        nh_private_.param("force_x", force_x_, 0.0);
        nh_private_.param("force_y", force_y_, 0.0);
        nh_private_.param("force_z", force_z_, 0.0);
    }

    void run() {
        ros::Rate rate(1000); // 1000 Hz loop rate

        while (ros::ok()) {
            // Prepare the force reference message
            geometry_msgs::Vector3 force_msg;
            force_msg.x = force_x_;
            force_msg.y = force_y_;
            force_msg.z = force_z_;

            // Publish the force reference
            force_ref_pub_.publish(force_msg);

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

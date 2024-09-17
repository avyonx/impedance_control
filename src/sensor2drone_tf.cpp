#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/TransformStamped.h>
#include <gazebo_msgs/GetLinkState.h>
#include <geometry_msgs/Pose.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "sensor2drone_node");
    ros::NodeHandle nh;

    // Publisher to publish the 4x4 transformation matrix
    ros::Publisher transform_pub = nh.advertise<std_msgs::Float64MultiArray>("force_sensor/transformation_matrix_input", 1);

    // Service client to call gazebo service /gazebo/get_link_state
    ros::ServiceClient get_link_state_client = nh.serviceClient<gazebo_msgs::GetLinkState>("/gazebo/get_link_state");

    // Create a request and response for the service call
    gazebo_msgs::GetLinkState get_link_state_srv;
    get_link_state_srv.request.link_name = "red::pole_tip"; // Modify as per your robot's link names
    get_link_state_srv.request.reference_frame = "red::red/base_link"; // Reference frame (previous joint or base link)

    // Define a Float64MultiArray for the 4x4 transformation matrix
    std_msgs::Float64MultiArray transform_matrix;
    transform_matrix.layout.dim.resize(2);
    transform_matrix.layout.dim[0].label = "rows";
    transform_matrix.layout.dim[0].size = 4;
    transform_matrix.layout.dim[0].stride = 4;
    transform_matrix.layout.dim[1].label = "cols";
    transform_matrix.layout.dim[1].size = 4;
    transform_matrix.layout.dim[1].stride = 1;
    transform_matrix.data.resize(16);

    ros::Rate rate(10);  // 10 Hz publish rate
    while (ros::ok()) {
        // Call the service to get the link state
        if (get_link_state_client.call(get_link_state_srv)) {
            if (get_link_state_srv.response.success) {
                // Get the pose from the service response
                geometry_msgs::Pose pose = get_link_state_srv.response.link_state.pose;
                double x = pose.position.x;
                double y = pose.position.y;
                double z = pose.position.z;
                
                tf2::Quaternion quat(
                    pose.orientation.x,
                    pose.orientation.y,
                    pose.orientation.z,
                    pose.orientation.w
                );

                // Convert quaternion to a rotation matrix
                tf2::Matrix3x3 rotation_matrix(quat);

                // Fill the matrix values (column-major order)
                // Rotation matrix components
                transform_matrix.data[0] = rotation_matrix[0][0];
                transform_matrix.data[1] = rotation_matrix[0][1];
                transform_matrix.data[2] = rotation_matrix[0][2];
                transform_matrix.data[3] = 0.0;  // last column (0)

                transform_matrix.data[4] = rotation_matrix[1][0];
                transform_matrix.data[5] = rotation_matrix[1][1];
                transform_matrix.data[6] = rotation_matrix[1][2];
                transform_matrix.data[7] = 0.0;  // last column (0)

                transform_matrix.data[8] = rotation_matrix[2][0];
                transform_matrix.data[9] = rotation_matrix[2][1];
                transform_matrix.data[10] = rotation_matrix[2][2];
                transform_matrix.data[11] = 0.0;  // last column (0)

                // Translation vector (position of the sensor in relation to the base)
                transform_matrix.data[12] = x;  // Translation x
                transform_matrix.data[13] = y;  // Translation y
                transform_matrix.data[14] = z;  // Translation z
                transform_matrix.data[15] = 1.0;  // Homogeneous coordinate

                // Publish the transformation matrix
                transform_pub.publish(transform_matrix);
            } else {
                ROS_ERROR("Failed to get link state: %s", get_link_state_srv.response.status_message.c_str());
            }
        } else {
            ROS_ERROR("Failed to call /gazebo/get_link_state service");
        }

        rate.sleep();
    }

    return 0;
}

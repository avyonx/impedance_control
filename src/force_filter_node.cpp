#include "ros/ros.h"

#include <eigen3/Eigen/Eigen>

#include <impedance_control/Tf1.h>
#include <impedance_control/median_filter.h>

#include <geometry_msgs/WrenchStamped.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_srvs/Empty.h>

#include <nav_msgs/Odometry.h>

#include <tf2/LinearMath/Matrix3x3.h>

class ForceFilter {
private:
    bool force_start_flag_, force_filters_initialized_, invert_;

    int moving_average_sample_number_, rate_;

    float *force_x_meas_;
    float *force_z_meas_;
    float *force_y_meas_;
    float *torque_x_meas_;
    float *torque_y_meas_;
    float *torque_z_meas_;

    float force_z_offset_, force_y_offset_, force_x_offset_, mrac_time_;
    float torque_y_offset_, torque_x_offset_, torque_z_offset_;    

    median_filter force_z_med_filt, force_x_med_filt, force_y_med_filt;

    Tf1 force_z_pt1_filt, force_x_pt1_filt, force_y_pt1_filt;

    Eigen::Matrix4d T_LG_, Ti_, T_force_sensor_;

    nav_msgs::Odometry curr_odom_;

public:
    ForceFilter(int rate, bool invert):
        rate_(rate),
        force_start_flag_(false),
        force_filters_initialized_(false),
        invert_(invert),
        force_x_offset_(0.0),
        force_y_offset_(0.0),
        force_z_offset_(0.0) {

        T_force_sensor_ <<  1,   0,  0,  0,
                            0,   1,  0,  0,
                            0,   0,  1,  0,
                            0,   0,  0,  1;

        T_LG_ << 1,  0,  0,  0,
                 0,  1,  0,  0,
                 0,  0,  1,  0,
                 0,  0,  0,  1;

        if (invert_) {
            Ti_ <<  -1,   0,   0,   0,
                     0,  -1,   0,   0,
                     0,   0,  -1,   0,
                     0,   0,   0,   1;
        }
        else {
            Ti_ <<   1,  0,  0,  0,
                     0,  1,  0,  0,
                     0,  0,  1,  0,
                     0,  0,  0,  1;
        }
    };

    ~ForceFilter() {
        delete[] force_z_meas_, force_y_meas_, force_x_meas_;
        delete[] torque_z_meas_, torque_y_meas_, torque_x_meas_;
    };

    void initForceFilters(int moving_average_sample_number, int median_filter_size, float pt1_t)
    {
        int i;
        force_filters_initialized_ = true;

        moving_average_sample_number_ = moving_average_sample_number;

        force_z_meas_ = new float[moving_average_sample_number];
        force_x_meas_ = new float[moving_average_sample_number];
        force_y_meas_ = new float[moving_average_sample_number];
        torque_x_meas_ = new float[moving_average_sample_number];
        torque_y_meas_ = new float[moving_average_sample_number];
        torque_z_meas_ = new float[moving_average_sample_number];

        force_z_med_filt.init(median_filter_size);
        force_x_med_filt.init(median_filter_size);
        force_y_med_filt.init(median_filter_size);

        force_z_pt1_filt.reset();
        force_z_pt1_filt.setNumerator(1.0, 0.0);
        force_z_pt1_filt.setDenominator(1, pt1_t);
        force_z_pt1_filt.c2d(1.0/rate_, "zoh");

        force_x_pt1_filt.reset();
        force_x_pt1_filt.setNumerator(1.0, 0.0);
        force_x_pt1_filt.setDenominator(1, pt1_t);
        force_x_pt1_filt.c2d(1.0/rate_, "zoh");

        force_y_pt1_filt.reset();
        force_y_pt1_filt.setNumerator(1.0, 0.0);
        force_y_pt1_filt.setDenominator(1, pt1_t);
        force_y_pt1_filt.c2d(1.0/rate_, "zoh");

        for (i = 0; i < moving_average_sample_number; i++)
        {
            force_z_meas_[i] = 0;
            force_x_meas_[i] = 0;
            force_y_meas_[i] = 0;
            torque_x_meas_[i] = 0;
            torque_y_meas_[i] = 0;
            torque_z_meas_[i] = 0;
        }
    };

    void getRotationTranslationMatrix(Eigen::Matrix4d &rotationTranslationMatrix, float *orientationEuler) {
        float r11, r12, r13, r21, r22, r23;
        float r31, r32, r33;

        float x, y, z;

        x = orientationEuler[0];
        y = orientationEuler[1];
        z = orientationEuler[2];

        r11 = cos(y)*cos(z);

        r12 = cos(z)*sin(x)*sin(y) - cos(x)*sin(z);

        r13 = sin(x)*sin(z) + cos(x)*cos(z)*sin(y);

        r21 = cos(y)*sin(z);

        r22 = cos(x)*cos(z) + sin(x)*sin(y)*sin(z);

        r23 = cos(x)*sin(y)*sin(z) - cos(z)*sin(x);

        r31 = -sin(y);

        r32 = cos(y)*sin(x);

        r33 = cos(x)*cos(y);

        rotationTranslationMatrix << r11, r12, r13, 0.0,
                                     r21, r22, r23, 0.0,
                                     r31, r32, r33, 0.0,
                                     0.0,   0.0,   0.0,   1.0;
    };

    void set_transform(float roll, float pitch, float yaw)
    {
        float euler[3];

        euler[0] = roll;
        euler[1] = pitch;
        euler[2] = yaw;

        getRotationTranslationMatrix(T_force_sensor_, euler);
    }

    void odomCb(const nav_msgs::Odometry& odom) {
       
      tf2::Matrix3x3 m(tf2::Quaternion(
            odom.pose.pose.orientation.x,
            odom.pose.pose.orientation.y,
            odom.pose.pose.orientation.z,
            odom.pose.pose.orientation.w));

      //T_LG_ << msg.data[0],  msg.data[1],  msg.data[2],  0.0,
      //         msg.data[4],  msg.data[5],  msg.data[6],  0.0,
      //         msg.data[8],  msg.data[9],  msg.data[10], 0.0,
      //         msg.data[12], msg.data[13], msg.data[14], msg.data[15];

      
      T_LG_ << m[0][0], m[0][1], m[0][2], 0.0, 
               m[1][0], m[1][1], m[1][2], 0.0,
               m[2][0], m[2][1], m[2][2], 0.0,
               0,       0,       0,       1.0;
    }

    void forceMeasurementGlobalCb(const geometry_msgs::WrenchStamped &msg)
    {
        if (!force_start_flag_) force_start_flag_ = true;

        for (int i=0; i<(moving_average_sample_number_-1); i++)
        {
            force_z_meas_[i] = force_z_meas_[i+1];
            force_x_meas_[i] = force_x_meas_[i+1];
            force_y_meas_[i] = force_y_meas_[i+1];
            torque_x_meas_[i] = torque_x_meas_[i+1];
            torque_y_meas_[i] = torque_y_meas_[i+1];
            torque_z_meas_[i] = torque_z_meas_[i+1];
        }

        force_z_meas_[moving_average_sample_number_-1] = force_z_pt1_filt.getDiscreteOutput(force_z_med_filt.filter(msg.wrench.force.z));
        force_x_meas_[moving_average_sample_number_-1] = force_x_pt1_filt.getDiscreteOutput(force_x_med_filt.filter(msg.wrench.force.x));
        force_y_meas_[moving_average_sample_number_-1] = force_y_pt1_filt.getDiscreteOutput(force_y_med_filt.filter(msg.wrench.force.y));
        torque_x_meas_[moving_average_sample_number_-1] = msg.wrench.torque.x;
        torque_y_meas_[moving_average_sample_number_-1] = msg.wrench.torque.y;
        torque_z_meas_[moving_average_sample_number_-1] = msg.wrench.torque.z;
    };

    void forceMeasurementLocalCb(const geometry_msgs::WrenchStamped &msg)
    {
        Eigen::Matrix4d F, Fg;
        Eigen::Matrix4d T, Tg;

        if (!force_start_flag_) force_start_flag_ = true;

        for (int i=0; i<(moving_average_sample_number_-1); i++)
        {
            force_z_meas_[i] = force_z_meas_[i+1];
            force_x_meas_[i] = force_x_meas_[i+1];
            force_y_meas_[i] = force_y_meas_[i+1];
            torque_x_meas_[i] = torque_x_meas_[i+1];
            torque_y_meas_[i] = torque_y_meas_[i+1];
            torque_z_meas_[i] = torque_z_meas_[i+1];
        }

        F << 1,  0,  0,  msg.wrench.force.x,
             0,  1,  0,  msg.wrench.force.y,
             0,  0,  1,  msg.wrench.force.z,
             0,  0,  0,  1;

        T << 1,  0,  0,  msg.wrench.torque.x,
             0,  1,  0,  msg.wrench.torque.y,
             0,  0,  1,  msg.wrench.torque.z,
             0,  0,  0,  1;

        Fg = Ti_ * T_LG_ * T_force_sensor_ * F;
        Tg = Ti_ * T_LG_ * T_force_sensor_ * T;

        force_z_meas_[moving_average_sample_number_-1] = force_z_pt1_filt.getDiscreteOutput(force_z_med_filt.filter(Fg(2, 3)));
        force_x_meas_[moving_average_sample_number_-1] = force_x_pt1_filt.getDiscreteOutput(force_x_med_filt.filter(Fg(0, 3)));
        force_y_meas_[moving_average_sample_number_-1] = force_y_pt1_filt.getDiscreteOutput(force_y_med_filt.filter(Fg(1, 3)));
        torque_x_meas_[moving_average_sample_number_-1] = Tg(0, 3);
        torque_y_meas_[moving_average_sample_number_-1] = Tg(1, 3);
        torque_z_meas_[moving_average_sample_number_-1] = Tg(2, 3);
    };

    void transformationMatrixCb(const std_msgs::Float64MultiArray &msg)
    {
        if (msg.data.size() == 16)
        {
            T_LG_ << msg.data[0],  msg.data[1],  msg.data[2],  0.0,
                     msg.data[4],  msg.data[5],  msg.data[6],  0.0,
                     msg.data[8],  msg.data[9],  msg.data[10], 0.0,
                     msg.data[12], msg.data[13], msg.data[14], msg.data[15];
        }
    }

    bool isReady(void)
    {
        return force_start_flag_ && force_filters_initialized_;
    };

    float getFilteredForceX(void) {
        float sum = 0;
        float average;

        for (int i = 0; i<moving_average_sample_number_; i++)
            sum += force_x_meas_[i];

        average = sum/moving_average_sample_number_ - force_x_offset_;

        return average;
    };

    float getFilteredForceY(void) {
        float sum = 0;
        float average;

        for (int i = 0; i<moving_average_sample_number_; i++)
            sum += force_y_meas_[i];

        average = sum/moving_average_sample_number_ - force_y_offset_;

        return average;
    };

    float getFilteredForceZ(void) {
        float sum = 0;
        float average;

        for (int i = 0; i<moving_average_sample_number_; i++)
            sum += force_z_meas_[i];

        average = sum/moving_average_sample_number_ - force_z_offset_;

        return average;
    };

    float getFilteredTorqueX(void) {
        float sum = 0;
        float average;

        for (int i = 0; i<moving_average_sample_number_; i++)
            sum += torque_x_meas_[i];

        average = sum/moving_average_sample_number_ - torque_x_offset_;

        return average;
    };

    float getFilteredTorqueY(void) {
        float sum = 0;
        float average;

        for (int i = 0; i<moving_average_sample_number_; i++)
            sum += torque_y_meas_[i];

        average = sum/moving_average_sample_number_ - torque_y_offset_;

        return average;
    };

    float getFilteredTorqueZ(void) {
        float sum = 0;
        float average;

        for (int i = 0; i<moving_average_sample_number_; i++)
            sum += torque_z_meas_[i];

        average = sum/moving_average_sample_number_ - torque_z_offset_;

        return average;
    };

    bool zeroAllCb(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res) {
        force_z_offset_ = 0.0;
        force_x_offset_ = 0.0;
        force_y_offset_ = 0.0;
        torque_x_offset_ = 0.0;
        torque_y_offset_ = 0.0;
        torque_z_offset_ = 0.0;

        force_z_offset_ = getFilteredForceZ();
        force_x_offset_ = getFilteredForceX();
        force_y_offset_ = getFilteredForceY();
        torque_x_offset_ = getFilteredTorqueX();
        torque_y_offset_ = getFilteredTorqueY();
        torque_z_offset_ = getFilteredTorqueZ();

        return true;
    };
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "force_filter_node");
    ros::NodeHandle private_node_handle_("~"), n;
    geometry_msgs::WrenchStamped filtered_ft_sensor_msg;
    int rate, masn, mfs, counter;
    float pt1_t, t_roll, t_pitch, t_yaw;
    bool invert_meas;

    private_node_handle_.param("rate", rate, int(1000));
    private_node_handle_.param("moving_average_sample_number", masn, int(10));
    private_node_handle_.param("median_filter_size", mfs, int(21));
    private_node_handle_.param("pt1_filter_time_constant", pt1_t, float(0.05));
    private_node_handle_.param("invert_force_measurements", invert_meas, bool(false));
    private_node_handle_.param("force_roll_transform", t_roll, float(0.0));
    private_node_handle_.param("force_pitch_transform", t_pitch, float(0.0));
    private_node_handle_.param("force_yaw_transform", t_yaw, float(0.0));

    ROS_INFO("Force pitch transform %.2f", t_pitch);
    ROS_INFO("Force roll transform %.2f", t_roll);
    ROS_INFO("Force yaw transform %.2f", t_yaw);
    ForceFilter filters(rate, invert_meas);

    filters.set_transform(t_roll, t_pitch, t_yaw);

    ros::Subscriber force_local_ros_sub = n.subscribe("force_sensor/force_torque_local_input", 1, &ForceFilter::forceMeasurementLocalCb, &filters);
    ros::Subscriber force_global_ros_sub = n.subscribe("force_sensor/force_torque_global_input", 1, &ForceFilter::forceMeasurementGlobalCb, &filters);
    ros::Subscriber transformation_matrix_ros_sub = n.subscribe("force_sensor/transformation_matrix_input", 1, &ForceFilter::transformationMatrixCb, &filters);
    ros::Subscriber odom_sub = n.subscribe("odom", 1, &ForceFilter::odomCb, &filters);
    
    ros::Publisher force_filtered_pub_ = n.advertise<geometry_msgs::WrenchStamped>("force_sensor/force_torque_output", 1);
    
    ros::ServiceServer zero_all_ros_srv = n.advertiseService("force_filter/zero_all", &ForceFilter::zeroAllCb, &filters);

    filters.initForceFilters(masn, mfs, pt1_t);

    ros::Rate loop_rate(rate);

    while (ros::Time::now().toSec() == 0 && ros::ok()) {
        ROS_INFO("Waiting for clock server to start 3");
        ros::Duration(0.5).sleep();
    }
    ROS_INFO("Received first clock message");

    while (!filters.isReady() && ros::ok()) {
        ros::spinOnce();

        ROS_INFO("Waiting for the first measurement.");
        ros::Duration(0.5).sleep();
    }
    ROS_INFO("Starting force filter node!");

    counter = 1;

    while (ros::ok())
    {
        ros::spinOnce();

        if ((counter % masn) == 0)
        {
            filtered_ft_sensor_msg.header.stamp = ros::Time::now();
            filtered_ft_sensor_msg.wrench.force.z = filters.getFilteredForceZ();
            filtered_ft_sensor_msg.wrench.force.y = filters.getFilteredForceY();
            filtered_ft_sensor_msg.wrench.force.x = filters.getFilteredForceX();
            filtered_ft_sensor_msg.wrench.torque.x = filters.getFilteredTorqueX();
            filtered_ft_sensor_msg.wrench.torque.y = filters.getFilteredTorqueY();
            filtered_ft_sensor_msg.wrench.torque.z = filters.getFilteredTorqueZ();
            force_filtered_pub_.publish(filtered_ft_sensor_msg);
            
            counter = 0;
        }

        counter++;
        loop_rate.sleep();
    }

    return 0;
}

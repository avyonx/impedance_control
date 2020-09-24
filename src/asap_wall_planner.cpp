#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <visualization_msgs/Marker.h>

template <class T> class TopicHandler {
public:
  TopicHandler(ros::NodeHandle &t_nh, const std::string &t_topicName,
               const double t_timeoutDuration = 2)
      : m_topicName(t_topicName), m_topicTimeout(t_timeoutDuration),
        m_lastMessgeTime(0), m_isResponsive(false), m_messageRecieved(false) {
    m_subT = t_nh.subscribe(m_topicName, 1, &TopicHandler::callback, this);

    if (t_timeoutDuration > 0) {
      m_watchdogTimer =
          t_nh.createTimer(ros::Duration(t_timeoutDuration),
                           &TopicHandler::watchdog_callback, this);
    }
  }

  const T &getData() { return m_data; }
  bool isResponsive() { return m_isResponsive; }
  bool isMessageRecieved() { return m_messageRecieved; }

private:
  void callback(const T &msg) {
    m_data = std::move(msg);
    m_lastMessgeTime = ros::Time::now().toSec();
    m_messageRecieved = true;
  }

  void watchdog_callback(const ros::TimerEvent & /* unused */) {
    double elapsedTime = ros::Time::now().toSec() - m_lastMessgeTime;
    if (elapsedTime > m_topicTimeout) {
      m_isResponsive = false;
      ROS_FATAL_STREAM("Topic: [" << m_subT.getTopic() << "] unresponsive.");
    } else {
      m_isResponsive = true;
    }
  }

  ros::Timer m_watchdogTimer;
  double m_topicTimeout, m_lastMessgeTime;
  bool m_isResponsive, m_messageRecieved;

  ros::Subscriber m_subT;
  T m_data;
  const std::string m_topicName;
};

enum planner_state { OFF, READY, EXECUTING };

static geometry_msgs::PoseStamped get_pose(const tf::Vector3 &translation,
                                           const tf::Quaternion &rotation) {
  geometry_msgs::PoseStamped p;
  p.pose.position.x = translation.x();
  p.pose.position.y = translation.y();
  p.pose.position.z = translation.z();
  p.pose.orientation.x = rotation.x();
  p.pose.orientation.y = rotation.y();
  p.pose.orientation.z = rotation.z();
  p.pose.orientation.w = rotation.w();
  return p;
}

static tf::Vector3 vector_from_pose(const geometry_msgs::Pose &pose) {
  return {pose.position.x, pose.position.y, pose.position.z};
}

static tf::Quaternion quaternion_from_pose(const geometry_msgs::Pose &pose) {
  return {pose.orientation.x, pose.orientation.y, pose.orientation.z,
          pose.orientation.w};
}

static geometry_msgs::PoseStamped
get_pose_offset(const geometry_msgs::PoseStamped &pose, double offset) {
  auto q = quaternion_from_pose(pose.pose);
  auto transformed =
      vector_from_pose(pose.pose) +
      tf::Vector3(-offset, 0, 0).rotate(q.getAxis(), q.getAngle());
  geometry_msgs::PoseStamped ret;
  ret.pose.position.x = transformed.x();
  ret.pose.position.y = transformed.y();
  ret.pose.position.z = transformed.z();
  ret.pose.orientation = pose.pose.orientation;
  return ret;
}

static visualization_msgs::Marker
get_marker(const std::vector<geometry_msgs::PoseStamped> &poses,
           std::string frame_id = "base_link") {
  visualization_msgs::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = ros::Time::now();
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.orientation.w = 1;
  marker.id = 1;
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  static constexpr double line_width = 0.025;
  marker.scale.x = line_width;
  marker.scale.y = line_width;
  marker.scale.z = line_width;
  marker.color.b = 1.0;
  marker.color.a = 1.0;

  for (const auto &pose : poses) {
    geometry_msgs::Point p;
    p.x = pose.pose.position.x;
    p.y = pose.pose.position.y;
    p.z = pose.pose.position.z;
    marker.points.push_back(p);
  }
  return marker;
}

static geometry_msgs::PoseStamped
get_world_pose(const geometry_msgs::PoseStamped &pose,
               const nav_msgs::Odometry &odom) {
  auto odom_q = quaternion_from_pose(odom.pose.pose);
  auto pose_tf =
      vector_from_pose(odom.pose.pose) +
      vector_from_pose(pose.pose).rotate(odom_q.getAxis(), odom_q.getAngle());
  auto orientation_tf = odom_q * quaternion_from_pose(pose.pose);

  // TODO: Link as parameter
  auto res = get_pose(pose_tf, orientation_tf);
  res.header.frame_id = "base_link";
  res.header.stamp = ros::Time::now();
  return res;
}

static std::vector<geometry_msgs::PoseStamped>
interpolate_points(const std::vector<geometry_msgs::PoseStamped> &input,
                   int interpolate_count = 50) {
  std::vector<geometry_msgs::PoseStamped> result;
  const double icount_double = static_cast<double>(interpolate_count);
  for (std::size_t i = 0; i < input.size() - 1; i++) {
    const auto current_pose = input.at(i);
    const auto next_pose = input.at(i + 1);

    for (int j = 0; j < interpolate_count; j++) {

      double ratio = j / icount_double;
      tf::Vector3 v;
      v.setInterpolate3(vector_from_pose(current_pose.pose),
                        vector_from_pose(next_pose.pose), ratio);
      auto q = tf::slerp(quaternion_from_pose(current_pose.pose),
                         quaternion_from_pose(next_pose.pose), ratio);
      result.emplace_back(get_pose(v, q));
    }
  }
  return result;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "wall_touching_planner");
  ros::NodeHandle nh;
  auto pose_handler =
      TopicHandler<geometry_msgs::PoseStamped>(nh, "plane/normal", 5);
  auto odom_handler = TopicHandler<nav_msgs::Odometry>(nh, "odometry");
  auto traj_visualizer =
      nh.advertise<visualization_msgs::Marker>("wall_path", 1);
  auto pose_ref = nh.advertise<geometry_msgs::PoseStamped>(
      "reference", 1);

  auto state = planner_state::OFF;
  std::vector<geometry_msgs::PoseStamped> trajPoints;

  // Setup planner
  using bool_req = std_srvs::SetBool::Request;
  using bool_resp = std_srvs::SetBool::Response;
  auto start_planner = nh.advertiseService<bool_req, bool_resp>(
      "plan_wall_path", [&](bool_req &request, bool_resp &response) {
        // Deactivate if requested
        if (!request.data) {
          state = planner_state::OFF;
          response.success = true;
          response.message = "Stop request accepted.";
          return true;
        }

        // Already active
        if (state == planner_state::EXECUTING) {
          response.success = false;
          response.message = "Trajectory is already executing.";
          return true;
        }

        // Dont activate if unable to find point or is inactive.
        if (!pose_handler.isResponsive()) {
          response.success = false;
          response.message = "Unable to plan wall path.";
          return true;
        }

        // Plan a new trajectory at this point
        trajPoints.clear();
        geometry_msgs::PoseStamped p;
        p.pose.orientation.w = 1;
        trajPoints.emplace_back(p);
        trajPoints.emplace_back(get_pose_offset(pose_handler.getData(), 2));
        trajPoints.emplace_back(get_pose_offset(pose_handler.getData(), 1.25));
        trajPoints = interpolate_points(trajPoints);
        traj_visualizer.publish(get_marker(trajPoints));

        state = planner_state::READY;
        response.success = true;
        response.message = "Wall path planned successfully.";
        return true;
      });

  // Setup executor
  using empty_req = std_srvs::Empty::Request;
  using empty_resp = std_srvs::Empty::Response;
  auto start_executing = nh.advertiseService<empty_req, empty_resp>(
      "execute_wall_path", [&](empty_req &, empty_resp &) {
        if (state != planner_state::READY) {
          ROS_FATAL("Wall trajectory is not ready.");
          return true;
        }

        if (trajPoints.empty()) {
          ROS_FATAL("Something went wrong... Trajectory is empty.");
          return true;
        }

        state = planner_state::EXECUTING;
        auto curr_odom = odom_handler.getData();

        // Make this a parameter
        static constexpr auto time_to_wall = 10.0;
        const double sleep_time = time_to_wall / trajPoints.size();
        for (const auto &pose : trajPoints) {
          pose_ref.publish(get_world_pose(pose, curr_odom));
          ROS_INFO("Sending reference.");
          ros::Duration(sleep_time).sleep();
        }
        state = planner_state::OFF;
        return true;
      });

  ros::spin();
  return 0;
}

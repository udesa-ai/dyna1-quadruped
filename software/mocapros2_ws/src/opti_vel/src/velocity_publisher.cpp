#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "mocap4r2_msgs/msg/rigid_bodies.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

using std::placeholders::_1;

// Angular velocity in body frame via the relative-rotation method:
//   q_delta = q_prev^-1 * q_curr   (rotation from k-1 to k, expressed in
//                                    the q_prev/body frame)
//   omega_body = rotvec(q_delta) / dt
// Exact for a constant-rate rotation over [k-1, k]; unlike differencing
// roll/pitch/yaw, it has no wraparound at +-180 deg and no gimbal lock.
tf2::Vector3 angularVelocityBody(const tf2::Quaternion & q_prev_in,
                                  const tf2::Quaternion & q_curr_in,
                                  double dt)
{
  tf2::Quaternion q_prev = q_prev_in;
  tf2::Quaternion q_curr = q_curr_in;

  // q and -q are the same orientation; force continuity with q_prev so
  // the relative rotation below comes out small instead of near-180 deg.
  if (q_prev.dot(q_curr) < 0.0) {
    q_curr = tf2::Quaternion(-q_curr.x(), -q_curr.y(), -q_curr.z(), -q_curr.w());
  }

  tf2::Quaternion q_delta = q_prev.inverse() * q_curr;

  double x = q_delta.x();
  double y = q_delta.y();
  double z = q_delta.z();
  double w = q_delta.w();
  double vnorm = std::sqrt(x * x + y * y + z * z);

  tf2::Vector3 rotvec;
  if (vnorm < 1e-12) {
    // small-angle: angle ~= 2*vnorm, axis ~= (x,y,z)/vnorm -> product ~= 2*(x,y,z)
    rotvec = tf2::Vector3(2.0 * x, 2.0 * y, 2.0 * z);
  } else {
    double angle = 2.0 * std::atan2(vnorm, w);   // in [0, pi]: shortest path
    double scale = angle / vnorm;
    rotvec = tf2::Vector3(x * scale, y * scale, z * scale);
  }

  return rotvec / dt;
}

class VelocityPublisher : public rclcpp::Node
{
public:
  VelocityPublisher()
  : Node("velocity_publisher")
  {
    subscription_ = this->create_subscription<mocap4r2_msgs::msg::RigidBodies>(
      "/rigid_bodies", 10,
      std::bind(&VelocityPublisher::topic_callback, this, _1));

    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "/rigid_body_velocity", 10);
  }

private:
  geometry_msgs::msg::Pose prev_pose_;
  rclcpp::Time prev_time_;
  bool has_prev_pose_ = false;

  rclcpp::Subscription<mocap4r2_msgs::msg::RigidBodies>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;

  void topic_callback(const mocap4r2_msgs::msg::RigidBodies::SharedPtr msg)
  {
    if (msg->rigidbodies.empty()) {
      RCLCPP_WARN(this->get_logger(), "No rigid bodies received.");
      return;
    }

    const auto & current_pose = msg->rigidbodies[0].pose;
    rclcpp::Time current_time = msg->header.stamp;

    if (has_prev_pose_) {
      double dt = (current_time - prev_time_).seconds();
      if (dt <= 0.0) return;

      // Compute position delta in world frame
      double dx = current_pose.position.x - prev_pose_.position.x;
      double dy = current_pose.position.y - prev_pose_.position.y;
      double dz = current_pose.position.z - prev_pose_.position.z;

      //dt = 0.01;
      // Velocity in world frame
      tf2::Vector3 vel_world(dx / dt, dy / dt, dz / dt);

      // Rotate velocity into body (base_link) frame
      tf2::Quaternion q_current;
      tf2::fromMsg(current_pose.orientation, q_current);
      tf2::Matrix3x3 rot_matrix(q_current);
      tf2::Vector3 vel_body = rot_matrix.transpose() * vel_world;

      geometry_msgs::msg::Twist twist;
      // clip linear velocity to maximum of +-2 m/s
      double max_linear_velocity = 2.0;
      vel_body.setX(std::clamp(vel_body.x(), -max_linear_velocity, max_linear_velocity));
      vel_body.setY(std::clamp(vel_body.y(), -max_linear_velocity, max_linear_velocity));
      vel_body.setZ(std::clamp(vel_body.z(), -max_linear_velocity, max_linear_velocity));
      twist.linear.x = vel_body.x();
      twist.linear.y = vel_body.y();
      twist.linear.z = vel_body.z();

      // Angular velocity: relative-rotation method (see angularVelocityBody)
      tf2::Quaternion q_prev, q_curr;
      tf2::fromMsg(prev_pose_.orientation, q_prev);
      tf2::fromMsg(current_pose.orientation, q_curr);

      tf2::Vector3 ang_vel_body = angularVelocityBody(q_prev, q_curr, dt);

      twist.angular.x = ang_vel_body.x();
      twist.angular.y = ang_vel_body.y();
      twist.angular.z = ang_vel_body.z();

      publisher_->publish(twist);
    }

    prev_pose_ = current_pose;
    prev_time_ = current_time;
    has_prev_pose_ = true;
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VelocityPublisher>());
  rclcpp::shutdown();
  return 0;
}

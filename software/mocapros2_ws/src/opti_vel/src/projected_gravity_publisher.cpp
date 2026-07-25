#include <functional>
#include <memory>
#include <fstream>
#include <chrono>
#include <iomanip>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "mocap4r2_msgs/msg/rigid_bodies.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

using std::placeholders::_1;

class ProjectedGravityPublisher : public rclcpp::Node
{
public:
  ProjectedGravityPublisher()
  : Node("projected_gravity_publisher")
  {
    subscription_ = this->create_subscription<mocap4r2_msgs::msg::RigidBodies>(
      "/rigid_bodies", 10,
      std::bind(&ProjectedGravityPublisher::topic_callback, this, _1));

    publisher_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>(
      "/projected_gravity_body", 10);

    init_csv();
  }

  ~ProjectedGravityPublisher()
  {
    if (csv_file_.is_open()) {
      csv_file_.close();
    }
  }

private:
  rclcpp::Subscription<mocap4r2_msgs::msg::RigidBodies>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr publisher_;
  std::ofstream csv_file_;
  std::string csv_filepath_;

  void init_csv()
  {
    auto now = std::chrono::system_clock::now();
    auto time = std::chrono::system_clock::to_time_t(now);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
      now.time_since_epoch()) % 1000;

    std::stringstream ss;
    ss << std::put_time(std::localtime(&time), "%Y%m%d_%H%M%S");
    std::string timestamp_str = ss.str();

    csv_filepath_ = std::string(std::getenv("HOME")) + "/projected_gravity_mocap_" + timestamp_str + ".csv";
    csv_file_.open(csv_filepath_, std::ios::out);

    if (csv_file_.is_open()) {
      csv_file_ << "timestamp,gravity_x,gravity_y,gravity_z\n";
      csv_file_.flush();
      RCLCPP_INFO(this->get_logger(), "Saving projected gravity to: %s", csv_filepath_.c_str());
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to open CSV file: %s", csv_filepath_.c_str());
    }
  }

  void topic_callback(const mocap4r2_msgs::msg::RigidBodies::SharedPtr msg)
  {
    if (msg->rigidbodies.empty()) {
      RCLCPP_WARN(this->get_logger(), "No rigid bodies received.");
      return;
    }

    const auto & current_pose = msg->rigidbodies[0].pose;

    tf2::Quaternion q_current;
    tf2::fromMsg(current_pose.orientation, q_current);
    tf2::Matrix3x3 rot_matrix(q_current);

    // Gravity in the world frame, expressed in meters per second squared.
    tf2::Vector3 gravity_world(0.0, 0.0, -9.81);

    // Project the world gravity vector into the body frame.
    // body_gravity = R_body_world^{T} * gravity_world.
    tf2::Vector3 gravity_body = rot_matrix.transpose() * gravity_world;

    geometry_msgs::msg::Vector3Stamped projected_msg;
    projected_msg.header.stamp = msg->header.stamp;
    projected_msg.header.frame_id = "body";
    projected_msg.vector.x = gravity_body.x();
    projected_msg.vector.y = gravity_body.y();
    projected_msg.vector.z = gravity_body.z();

    publisher_->publish(projected_msg);

    // Save to CSV
    if (csv_file_.is_open()) {
      double timestamp = msg->header.stamp.sec + msg->header.stamp.nanosec / 1e9;
      csv_file_ << std::fixed << std::setprecision(6)
                << timestamp << ","
                << gravity_body.x() << ","
                << gravity_body.y() << ","
                << gravity_body.z() << "\n";
      csv_file_.flush();
    }
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ProjectedGravityPublisher>());
  rclcpp::shutdown();
  return 0;
}

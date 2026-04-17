#include <memory>
#include <string>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"
#include "unitree_go/msg/low_state.hpp"
#include "unitree_go/msg/sport_mode_state.hpp"
#include "sensor_msgs/msg/image.hpp"

class A2Bridge : public rclcpp::Node {
public:
  explicit A2Bridge(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("a2_bridge_node", options),
    tf_broadcaster_(this),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    joint_states_pub_    = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
    imu_pub_             = this->create_publisher<sensor_msgs::msg::Imu>("/imu/data", 10);
    image_pub_           = this->create_publisher<sensor_msgs::msg::Image>("/camera/image_raw", 10);
    clock_pub_           = this->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);
    odom_pub_            = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    state_est_pub_       = this->create_publisher<nav_msgs::msg::Odometry>("/state_estimation", 10);
    registered_scan_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/registered_scan", 10);

    lowstate_sub_ = this->create_subscription<unitree_go::msg::LowState>(
      "/lowstate", 10,
      std::bind(&A2Bridge::lowstate_callback, this, std::placeholders::_1));

    sportstate_sub_ = this->create_subscription<unitree_go::msg::SportModeState>(
      "/sportmodestate", 10,
      std::bind(&A2Bridge::sportstate_callback, this, std::placeholders::_1));

    mujoco_camera_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/mujoco/front_camera_pointcloud", 10,
      std::bind(&A2Bridge::camera_callback, this, std::placeholders::_1));

    front_lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/mujoco/front_lidar", 10,
      std::bind(&A2Bridge::lidar_callback, this, std::placeholders::_1));

    joint_names_ = {
      "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
      "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
      "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
      "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint"
    };
  }

private:
  void lowstate_callback(const unitree_go::msg::LowState::SharedPtr msg)
  {
    // --- sim clock from tick (milliseconds -> nanoseconds) ---
    uint64_t tick_ns = static_cast<uint64_t>(msg->tick) * 1000000ULL;
    auto clock_msg = rosgraph_msgs::msg::Clock();
    clock_msg.clock.sec     = static_cast<int32_t>(tick_ns / 1000000000ULL);
    clock_msg.clock.nanosec = static_cast<uint32_t>(tick_ns % 1000000000ULL);
    clock_pub_->publish(clock_msg);

    auto stamp = clock_msg.clock;

    // --- joint states ---
    auto joint_states_msg = sensor_msgs::msg::JointState();
    joint_states_msg.header.stamp = stamp;
    joint_states_msg.name = joint_names_;
    for (size_t i = 0; i < joint_names_.size(); ++i) {
      joint_states_msg.position.push_back(msg->motor_state[i].q);
      joint_states_msg.velocity.push_back(msg->motor_state[i].dq);
      joint_states_msg.effort.push_back(msg->motor_state[i].tau_est);
    }
    joint_states_pub_->publish(joint_states_msg);

    // --- IMU ---
    auto imu_msg = sensor_msgs::msg::Imu();
    imu_msg.header.stamp    = stamp;
    imu_msg.header.frame_id = "imu_link";
    imu_msg.orientation.w = msg->imu_state.quaternion[0];
    imu_msg.orientation.x = msg->imu_state.quaternion[1];
    imu_msg.orientation.y = msg->imu_state.quaternion[2];
    imu_msg.orientation.z = msg->imu_state.quaternion[3];
    imu_msg.angular_velocity.x = msg->imu_state.gyroscope[0];
    imu_msg.angular_velocity.y = msg->imu_state.gyroscope[1];
    imu_msg.angular_velocity.z = msg->imu_state.gyroscope[2];
    imu_msg.linear_acceleration.x = msg->imu_state.accelerometer[0];
    imu_msg.linear_acceleration.y = msg->imu_state.accelerometer[1];
    imu_msg.linear_acceleration.z = msg->imu_state.accelerometer[2];
    imu_pub_->publish(imu_msg);

    // Cache latest stamp and orientation for use in sportstate_callback
    last_stamp_ = stamp;
    last_quat_[0] = msg->imu_state.quaternion[0];  // w
    last_quat_[1] = msg->imu_state.quaternion[1];  // x
    last_quat_[2] = msg->imu_state.quaternion[2];  // y
    last_quat_[3] = msg->imu_state.quaternion[3];  // z
    last_ang_vel_[0] = msg->imu_state.gyroscope[0];
    last_ang_vel_[1] = msg->imu_state.gyroscope[1];
    last_ang_vel_[2] = msg->imu_state.gyroscope[2];
  }

  void sportstate_callback(const unitree_go::msg::SportModeState::SharedPtr msg)
  {
    // Ground truth pose and velocity from MuJoCo via the Unitree highstate
    auto stamp = last_stamp_;

    // --- Odometry ---
    auto odom_msg = nav_msgs::msg::Odometry();
    odom_msg.header.stamp    = stamp;
    odom_msg.header.frame_id = "map";
    odom_msg.child_frame_id  = "base_link";

    odom_msg.pose.pose.position.x = msg->position[0];
    odom_msg.pose.pose.position.y = msg->position[1];
    odom_msg.pose.pose.position.z = msg->position[2];
    odom_msg.pose.pose.orientation.w = last_quat_[0];
    odom_msg.pose.pose.orientation.x = last_quat_[1];
    odom_msg.pose.pose.orientation.y = last_quat_[2];
    odom_msg.pose.pose.orientation.z = last_quat_[3];

    odom_msg.twist.twist.linear.x  = msg->velocity[0];
    odom_msg.twist.twist.linear.y  = msg->velocity[1];
    odom_msg.twist.twist.linear.z  = msg->velocity[2];
    odom_msg.twist.twist.angular.x = last_ang_vel_[0];
    odom_msg.twist.twist.angular.y = last_ang_vel_[1];
    odom_msg.twist.twist.angular.z = last_ang_vel_[2];

    odom_pub_->publish(odom_msg);
    // Also publish as /state_estimation for navigation stack
    state_est_pub_->publish(odom_msg);

    // --- TF: odom -> base_link ---
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp    = stamp;
    tf.header.frame_id = "map";
    tf.child_frame_id  = "base_link";
    tf.transform.translation.x = msg->position[0];
    tf.transform.translation.y = msg->position[1];
    tf.transform.translation.z = msg->position[2];
    tf.transform.rotation.w = last_quat_[0];
    tf.transform.rotation.x = last_quat_[1];
    tf.transform.rotation.y = last_quat_[2];
    tf.transform.rotation.z = last_quat_[3];
    tf_broadcaster_.sendTransform(tf);
  }

  void camera_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    auto image_msg = sensor_msgs::msg::Image();
    image_msg.header.stamp    = msg->header.stamp;
    image_msg.header.frame_id = "camera_link";
    image_msg.height     = msg->height;
    image_msg.width      = msg->width;
    image_msg.encoding   = "rgb8";
    image_msg.is_bigendian = false;
    image_msg.step       = msg->row_step;
    image_msg.data       = msg->data;
    image_pub_->publish(image_msg);
  }

  void lidar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    // Transform lidar cloud from front_lidar_link -> odom frame for navigation stack
    try {
      auto transform = tf_buffer_.lookupTransform(
        "map", msg->header.frame_id, tf2::TimePointZero);
      sensor_msgs::msg::PointCloud2 cloud_odom;
      tf2::doTransform(*msg, cloud_odom, transform);
      cloud_odom.header.stamp    = msg->header.stamp;
      cloud_odom.header.frame_id = "map";

      // Add 'intensity' field alias for 'dist' so terrain_analysis can parse
      // the cloud as PointXYZI without warnings. No data copy — same offset.
      uint32_t dist_offset = 12;  // default offset of 'dist' in our format
      for (const auto & f : cloud_odom.fields) {
        if (f.name == "dist") { dist_offset = f.offset; break; }
      }
      sensor_msgs::msg::PointField intensity_field;
      intensity_field.name     = "intensity";
      intensity_field.offset   = dist_offset;
      intensity_field.datatype = sensor_msgs::msg::PointField::FLOAT32;
      intensity_field.count    = 1;
      cloud_odom.fields.push_back(intensity_field);

      registered_scan_pub_->publish(cloud_odom);
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "Lidar TF lookup failed: %s", e.what());
    }
  }

  // Subscribers
  rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr        lowstate_sub_;
  rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr  sportstate_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr    mujoco_camera_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr    front_lidar_sub_;

  // Publishers
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr  joint_states_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr         imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr       image_pub_;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr     clock_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr       odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr       state_est_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr registered_scan_pub_;

  tf2_ros::TransformBroadcaster tf_broadcaster_;
  tf2_ros::Buffer               tf_buffer_;
  tf2_ros::TransformListener    tf_listener_;

  std::vector<std::string> joint_names_;

  // Cached state from lowstate (used in sportstate_callback)
  builtin_interfaces::msg::Time last_stamp_{};
  float last_quat_[4]    = {1, 0, 0, 0};  // w, x, y, z
  float last_ang_vel_[3] = {0, 0, 0};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<A2Bridge>());
  rclcpp::shutdown();
  return 0;
}

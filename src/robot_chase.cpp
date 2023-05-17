#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcutils/logging.h"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"


class TfFrameListener : public rclcpp::Node {
public:
  TfFrameListener() : Node("morty_tf_frame_listener") {
    RCLCPP_INFO_STREAM(this->get_logger(), "TfFrameListener created");

    // Set log level
    auto log_answer = rcutils_logging_set_logger_level(
        this->get_logger().get_name(), RCUTILS_LOG_SEVERITY_INFO);
    (void)log_answer;

    // Setup tf listener
    this->tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    this->tf_listener =
        std::make_shared<tf2_ros::TransformListener>(*this->tf_buffer);

    // Setup timer
    this->timer =
        this->create_wall_timer(std::chrono::milliseconds(500),
                                std::bind(&TfFrameListener::timer_clb, this));

    // Setup publisher
    this->pub_cmd_vel =
        this->create_publisher<geometry_msgs::msg::Twist>("rick/cmd_vel", 3);
  }

private:
  std::string target_frame = "morty/base_link";
  std::string reference_frame = "rick/base_link";

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr
      pub_cmd_vel;                    // Publihser to control robot
  rclcpp::TimerBase::SharedPtr timer; // Timer

  std::unique_ptr<tf2_ros::Buffer> tf_buffer;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener;

  void timer_clb() {

    geometry_msgs::msg::TransformStamped t;

    // Get transformation between target and reference frame
    try {
      t = this->tf_buffer->lookupTransform(
          this->reference_frame, this->target_frame, tf2::TimePointZero);
    } catch (const tf2::TransformException &ex) {
      RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s",
                  reference_frame.c_str(), target_frame.c_str(), ex.what());
      return;
    }

    geometry_msgs::msg::Twist msg;

    static const float kp_yaw = 0.5;
    static const float kp_distance = 0.2;

    // Calculate distance and angle rrors
    float error_yaw =
        atan2(t.transform.translation.y, t.transform.translation.x);
    float error_distance = sqrt(pow(t.transform.translation.x, 2) +
                                pow(t.transform.translation.y, 2));

    msg.angular.z = kp_yaw * error_yaw;
    msg.linear.x = kp_distance * error_distance;

    this->pub_cmd_vel->publish(msg);

    RCLCPP_DEBUG(this->get_logger(), "Dist: %.2f, %.2f",
                 t.transform.translation.x, t.transform.translation.y);

    return;
  }
};

int main(int argc, char *argv[]) {

  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<TfFrameListener>());

  return 0;
}
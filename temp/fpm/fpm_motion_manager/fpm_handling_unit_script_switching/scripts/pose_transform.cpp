#include "rclcpp/rclcpp.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include <std_msgs/msg/float32.hpp>
#include "std_msgs/msg/string.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <std_srvs/srv/trigger.hpp>
#include <ur_msgs/srv/set_io.hpp>
using namespace std::chrono_literals;
class MinimalSubscriber : public rclcpp::Node
{
  public:
    MinimalSubscriber()
    : Node("minimal_subscriber")
    {
      target_frame_ = this->declare_parameter<std::string>("target_frame", "wrist_3_link");
      //initial_frame_ = this->declare_parameter<std::string>("initial_frame", "wrist_3_link");
      tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
      timer_ = this->create_wall_timer(
      0.1s, std::bind(&MinimalSubscriber::on_timer, this));
      
      start_pose.x = -0.000;
      start_pose.y = -0.720;
      start_pose.z = -0.001;
    }

  private:
    void on_timer(){
      std::string fromFrameRel = target_frame_.c_str();
      std::string toFrameRel = "base_link_inertia";
      geometry_msgs::msg::TransformStamped t;
      t = tf_buffer_->lookupTransform(
              toFrameRel, fromFrameRel,
              tf2::TimePointZero);
      RCLCPP_INFO(this->get_logger(), "x: '%.3f'", t.transform.translation.x);
      RCLCPP_INFO(this->get_logger(), "y: '%.3f'", t.transform.translation.y);
      RCLCPP_INFO(this->get_logger(), "z: '%.3f'", t.transform.translation.z);
      RCLCPP_INFO(this->get_logger(), "rx: '%.3f'", t.transform.rotation.x);
      RCLCPP_INFO(this->get_logger(), "ry: '%.3f'", t.transform.rotation.y);
      RCLCPP_INFO(this->get_logger(), "rz: '%.3f'", t.transform.rotation.z);
      RCLCPP_INFO(this->get_logger(), "rw: '%.3f'", t.transform.rotation.w);

      //distance = sqrt(pow(current_pose.x - start_pose.x, 2)
        //        + pow(current_pose.y - start_pose.y, 2) + pow(current_pose.z-start_pose.z, 2));
      //RCLCPP_INFO(this->get_logger(), "The distance is %.3f", distance);
      
    }
    
      std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
      std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
      rclcpp::TimerBase::SharedPtr timer_{nullptr};
      std::string target_frame_;
      geometry_msgs::msg::Point current_pose;
      geometry_msgs::msg::Point start_pose;
      float distance;
      
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}

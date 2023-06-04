#include "attach_shelf/srv/detail/go_to_loading__struct.hpp"
#include "attach_shelf/srv/go_to_loading.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/rate.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/subscription.hpp"
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
#include "std_msgs/msg/detail/empty__struct.hpp"
#include "std_msgs/msg/empty.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include <cmath>
#include <functional>
#include <iostream>
#include <memory>
#include <tf2_ros/transform_listener.h>
#include <vector>
#define PI 3.141592653589793238462643
using attach = attach_shelf::srv::GoToLoading;
using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

class ServerNode : public rclcpp::Node {
public:
  ServerNode() : Node("service_moving") {
    // transfor broadcaster
    broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    // data laser_scan
    laser_data = std::make_shared<sensor_msgs::msg::LaserScan>();
    // services
    srv_ = create_service<attach>(
        "/approach_shelf",
        std::bind(&ServerNode::approach_shelf_callback, this, _1, _2));
    // publisher
    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("robot/cmd_vel", 10);
    pub_elevator_up =
        this->create_publisher<std_msgs::msg::Empty>("/elevator_up", 10);
    // subcrition
    sub_laser = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 1, std::bind(&ServerNode::laserCallback, this, _1));

    // timer for control
    timer_cb_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    timer_ = this->create_wall_timer(
        100ms, std::bind(&ServerNode::tracking_transfor, this),
        timer_cb_group_);
  }

private:
  rclcpp::Service<attach>::SharedPtr srv_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr pub_elevator_up;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_laser;
  std::shared_ptr<sensor_msgs::msg::LaserScan> laser_data;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::CallbackGroup::SharedPtr timer_cb_group_;
  // vector for detection legs
  std::vector<int> index_legs;
  // flags for control
  bool flag_timer = false;
  bool flag_point_goal = false;
  // laser callback
  void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    *laser_data = *msg;
    // RCLCPP_INFO(this->get_logger(), "Laser scan arrived");
  }
  // services callback
  void
  approach_shelf_callback(const std::shared_ptr<attach::Request> request,
                          const std::shared_ptr<attach::Response> response) {
    if (request->attach_to_shelf == false) {
      if (calculate_index_legs()) {
        publish_transfor();
        index_legs.clear();
        response->complete = true;
        RCLCPP_INFO(this->get_logger(), "Service is called only publish frame");
      } else {
        response->complete = false;
        index_legs.clear();
        RCLCPP_INFO(this->get_logger(),
                    "Service is called, dont found legs enviorment");
      }
      // rclcpp::shutdown();
    } else {
      if (calculate_index_legs()) {
        publish_transfor();
        flag_timer = true;
        while (timer_->is_canceled() == false) {
        }
        geometry_msgs::msg::Twist msg;
        msg.linear.x = 0.45;
        rclcpp::Duration duration(2s);
        rclcpp::Time startTime = this->now();

        rclcpp::Rate r(20);
        while (this->now() - startTime < duration) {
          publisher_->publish(msg);
          r.sleep();
        }
        msg.linear.x = 0.0;
        publisher_->publish(msg);
        // up elevator
        std_msgs::msg::Empty msg_empty;
        pub_elevator_up->publish(msg_empty);
        index_legs.clear();
        response->complete = true;
        RCLCPP_INFO(this->get_logger(), "Service finished successfully");
        // rclcpp::shutdown();
      } else {
        index_legs.clear();
        response->complete = false;
        index_legs.clear();
        RCLCPP_INFO(this->get_logger(), "Service is called, dont found legs");

        // rclcpp::shutdown();
      }
    }
  }

  void tracking_transfor() {
    if (flag_timer == false) {
    } else {
      std::string fromFrameRel = "robot_front_laser_base_link";
      std::string toFrameRel = "cart_frame";
      geometry_msgs::msg::TransformStamped t;
      try {
        t = tf_buffer_->lookupTransform(fromFrameRel, toFrameRel,
                                        tf2::TimePointZero);
      } catch (const tf2::TransformException &ex) {
        RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s",
                    toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
        return;
      }
      geometry_msgs::msg::Twist msg;
      // constans velocity
      const float kp_yaw = 0.2;
      const float kp_distance = 0.4;
      double roll, pitch, yaw;
      // errors
      float error_distance = sqrt(pow(t.transform.translation.x, 2) +
                                  pow(t.transform.translation.y, 2));
      float error_yaw =
          atan2(t.transform.translation.y, t.transform.translation.x);

      tf2::Quaternion quaternion(t.transform.rotation.x, t.transform.rotation.y,
                                 t.transform.rotation.z,
                                 t.transform.rotation.w);
      tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
      float error_yaw_final = yaw;

      // logic
      if (flag_point_goal == false) {
        if (error_distance > 0.01 || abs(error_yaw) > 0.01) {
          // angle negative -> turn right , angle positive -> turn left
          msg.angular.z = -1 * kp_yaw * error_yaw;
          // saturation low
          if (abs(msg.angular.z) < 0.07) {
            if (msg.angular.z < 0) {
              msg.angular.z = -0.07;
            } else {
              msg.angular.z = 0.07;
            }
          }
          RCLCPP_INFO(this->get_logger(),
                      "Error distance [%.3f] Error yaw [%.3f] ", error_distance,
                      error_yaw);
          // error for not collisions
          if (error_distance > 0.01) {
            msg.linear.x = kp_distance * error_distance;
            if (abs(msg.linear.x) < 0.06) {
              if (msg.linear.x < 0) {
                msg.linear.x = -0.06;
              } else {
                msg.linear.x = 0.06;
              }
            }
          } else {
            msg.angular.z = 0.0;
            msg.linear.x = 0.0;
            flag_point_goal = true;
          }
        } else {
          flag_point_goal = true;
        }
      }
      // robot stop
      else {
        msg.linear.x = 0;
        if (abs(error_yaw_final) > 0.02) {
          RCLCPP_INFO(this->get_logger(), "Error_yaw_final [%.3f] ",
                      error_yaw_final);
          msg.angular.z = -1 * kp_yaw * error_yaw_final;
          if (abs(msg.angular.z) < 0.07) {
            if (msg.angular.z < 0) {
              msg.angular.z = -0.07;
            } else {
              msg.angular.z = 0.07;
            }
          }
        } else {
          msg.angular.z = -0.00;
          flag_timer = false;
          timer_->cancel();
        }
      }
      // Saturations vel
      if (msg.linear.x > 2) {
        msg.linear.x = 2;
      }
      if (abs(msg.angular.z) > 3.14) {
        if (error_yaw > 0) {
          msg.angular.z = -3.14;
        } else {
          msg.angular.z = 3.14;
        }
      }
      RCLCPP_INFO(this->get_logger(), "Vel x [%.3f] Vel theta [%.3f] ",
                  msg.linear.x, msg.angular.z);
      publisher_->publish(msg);
    }
  }
  void publish_transfor() {
    float ptr_data[3];
    calculate_point_middle(ptr_data);

    RCLCPP_INFO(this->get_logger(), "dt [%.3f]", ptr_data[0]);
    RCLCPP_INFO(this->get_logger(), "theta_total [%.3f]", ptr_data[1]);
    float x;
    float y;
    float z;
    double roll, pitch, yaw;
    try {
      geometry_msgs::msg::TransformStamped t;

      t = tf_buffer_->lookupTransform("robot_odom", "robot_front_laser_link",
                                      tf2::TimePointZero);
      x = t.transform.translation.x;
      y = t.transform.translation.y;
      z = t.transform.translation.z;
      // Obtener los valores de traslación y rotación

      tf2::Quaternion quaternion(t.transform.rotation.x, t.transform.rotation.y,
                                 t.transform.rotation.z,
                                 t.transform.rotation.w);
      tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);

      // Imprimir los valores obtenidos
      RCLCPP_INFO(this->get_logger(), "Transform from frame1 to frame2:");
      RCLCPP_INFO(this->get_logger(), "Translation: x=%.2f, y=%.2f, z=%.2f", x,
                  y, z);
      RCLCPP_INFO(this->get_logger(),
                  "Rotation: roll=%.2f, pitch=%.2f, yaw=%.2f", roll, pitch,
                  yaw);
    } catch (tf2::TransformException &ex) {
      RCLCPP_ERROR(this->get_logger(), "Failed to get transform: %s",
                   ex.what());
    }

    geometry_msgs::msg::TransformStamped transform_;
    transform_.header.frame_id = "robot_odom"; // Marco de referencia fuente
    transform_.child_frame_id = "cart_frame";  // Marco de referencia objetivo

    // Definir la transformación estática (traslación y rotación)
    /*transform_.transform.translation.x = 0;
    transform_.transform.translation.y = 0;*/
    transform_.transform.translation.x =
        x + ptr_data[0] * std::sin(ptr_data[1]) * std::sin(yaw) +  ptr_data[0] * std::cos(ptr_data[1]) * std::cos(yaw) ;
    transform_.transform.translation.y =
        y + ptr_data[0] * std::cos(ptr_data[1])* std::sin(yaw)  - ptr_data[0] * std::sin(ptr_data[1])* std::cos(yaw);

    transform_.transform.translation.z = z;

    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw - ptr_data[2]); // Sin rotación
    transform_.transform.rotation.x = q.x();
    transform_.transform.rotation.y = q.y();
    transform_.transform.rotation.z = q.z();
    transform_.transform.rotation.w = q.w();

    // Publicar la transformación estática
    broadcaster_->sendTransform(transform_);
  }

  void calculate_point_middle(float *ptr) {
    float angle_increment = laser_data->angle_increment;
    float angle_base = laser_data->angle_min;
    float index_leg_1 = index_legs.at(4);
    float index_leg_2 = index_legs.at(5);
    float theta_a_leg = (index_leg_1 * angle_increment + angle_base);
    float theta_b_leg = (index_leg_2 * angle_increment + angle_base);
    RCLCPP_INFO(this->get_logger(), "theta_a_leg [%.3f]", theta_a_leg);
    RCLCPP_INFO(this->get_logger(), "theta_b_leg [%.3f]", theta_b_leg);
    float d1 = laser_data->ranges[index_leg_1];
    float d2 = laser_data->ranges[index_leg_2];
    RCLCPP_INFO(this->get_logger(), "d1 [%.3f]", d1);
    RCLCPP_INFO(this->get_logger(), "d2 [%.3f]", d2);
    float dt_alfa = d1 * std::cos(theta_a_leg) + d2 * std::cos(theta_b_leg);
    float dt_beta = d1 * std::sin(theta_a_leg) + d2 * std::sin(theta_b_leg);
    RCLCPP_INFO(this->get_logger(), "dt_alfa [%.3f]", dt_alfa);
    RCLCPP_INFO(this->get_logger(), "dt_beta [%.3f]", dt_beta);
    float dt = std::sqrt(std::pow(dt_alfa, 2) + std::pow(dt_beta, 2)) / 2;
    float theta_total = std::atan2(dt_beta, dt_alfa);
    RCLCPP_INFO(this->get_logger(), "dt [%.3f]", dt);
    RCLCPP_INFO(this->get_logger(), "theta_total [%.3f]", theta_total);
    float c = std::sqrt(std::pow(d2, 2) + std::pow(dt, 2) -
                        2 * d2 * dt * std::cos(abs(theta_total - theta_b_leg)));
    RCLCPP_INFO(this->get_logger(), "c [%.3f]", c);
    float alfa_theta = abs(std::acos(
        (std::pow(c, 2) + std::pow(dt, 2) - std::pow(d2, 2)) / (2 * c * dt)));
    RCLCPP_INFO(this->get_logger(), "alfa_theta [%.3f]", alfa_theta);
    float theta_final = PI / 2 + theta_total - alfa_theta;
    RCLCPP_INFO(this->get_logger(), "theta_final [%.3f]", theta_final);
    ptr[0] = dt;
    ptr[1] = theta_total;
    ptr[2] = theta_final;
  }
  bool calculate_index_legs() {
    int count = 0;
    bool flag_up = false;
    bool flag_down = false;
    std::vector<float> intensities = laser_data->intensities;
    RCLCPP_INFO(this->get_logger(), "Size [%ld] ", intensities.size());
    for (auto item = intensities.begin(); item != intensities.end(); item++) {
      if (*item == 0 && flag_up == false && flag_down == false) {
        flag_up = true;
        flag_down = false;

      } else if (flag_up == true && *item != 0) {
        if (item == intensities.end()) {
          index_legs.push_back(item - intensities.begin());
          count++;

          flag_up = false;
          flag_down = false;
        }
        index_legs.push_back(item - intensities.begin());
        flag_down = true;
        flag_up = false;

      } else if (flag_down == true && *item == 0) {
        count++;
        index_legs.push_back(item - intensities.begin());
        flag_up = true;
        flag_down = false;

      }

      else if (*item != 0 && flag_up == false && flag_down == false) {
        flag_up = false;
        flag_down = true;
      }
    }
    if (count == 2) {
      RCLCPP_INFO(this->get_logger(), "found[%d] legs ", count);
      index_legs.push_back(int((index_legs.at(0) + index_legs.at(1)) / 2));
      index_legs.push_back(int((index_legs.at(2) + index_legs.at(3)) / 2));
      RCLCPP_INFO(this->get_logger(), " [%d] [%d] [%d] [%d]", index_legs.at(0),
                  index_legs.at(1), index_legs.at(2), index_legs.at(3));
      RCLCPP_INFO(this->get_logger(),
                  "Leg 1 there is [%d] Leg 2 there is [%d] ", index_legs.at(4),
                  index_legs.at(5));
      return true;
    } else {
      RCLCPP_INFO(this->get_logger(), "found [%d] legs ", count);
      return false;
    }
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  // Instantiate a node.
  rclcpp::Node::SharedPtr node = std::make_shared<ServerNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
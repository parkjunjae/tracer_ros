/*
 * tracer_messenger.hpp
 *
 * Created on: Jun 14, 2022 16:37
 * Description:
 *
 * Copyright (c) 2022 tx (tx)
 */

#ifndef TRACER_MESSENGER_HPP
#define TRACER_MESSENGER_HPP

#include <string>
#include <mutex>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "tracer_msgs/msg/tracer_status.hpp"
#include "tracer_msgs/msg/tracer_light_cmd.hpp"
#include "tracer_msgs/msg/tracer_rc_state.hpp"

#include "ugv_sdk/mobile_robot/tracer_robot.hpp"
#include "ugv_sdk/utilities/protocol_detector.hpp"
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/transform_datatypes.hpp>
#include <tf2/utils.h>

namespace westonrobot
{
  template <typename TracerType>
  class TracerMessenger
  {
  public:
    TracerMessenger(std::shared_ptr<TracerType> tracer, rclcpp::Node *node)
        : tracer_(tracer), node_(node)
    {

      position_x_ = 0.0;
      position_y_ = 0.0;
      theta_ = 0.0;
    }

    void SetOdometryFrame(std::string frame) { odom_frame_ = frame; }
    void SetBaseFrame(std::string frame) { base_frame_ = frame; }
    void SetOdometryTopicName(std::string name) { odom_topic_name_ = name; }

    void SetSimulationMode(int loop_rate)
    {
      simulated_robot_ = true;
      sim_control_rate_ = loop_rate;
    }

    void SetupSubscription()
    {
      // odometry publisher
      odom_pub_ =
          node_->create_publisher<nav_msgs::msg::Odometry>(odom_topic_name_, 50);
      status_pub_ = node_->create_publisher<tracer_msgs::msg::TracerStatus>(
          "/tracer_status", 10);
      rc_status_pub_ = node_->create_publisher<tracer_msgs::msg::TracerRCState>(
          "/tracer_rc_status", 10);

      // slam에서 값 가져오기
      slam_odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
          "/aft_mapped_to_init", 10,
          std::bind(&TracerMessenger::SlamOdomCallback, this, std::placeholders::_1));

      // cmd subscriber
      motion_cmd_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
          "/cmd_vel", 5,
          std::bind(&TracerMessenger::TwistCmdCallback, this,
                    std::placeholders::_1));
      light_cmd_sub_ = node_->create_subscription<tracer_msgs::msg::TracerLightCmd>(
          "/light_control", 5,
          std::bind(&TracerMessenger::LightCmdCallback, this,
                    std::placeholders::_1));

      tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);
    }

    void SlamOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
      if (!slam_heading_initialized_)
      {
        double YAW_OFFSET = 0.1;
        double yaw = tf2::getYaw(msg->pose.pose.orientation);
        theta_ = yaw + YAW_OFFSET;
        position_x_ = msg->pose.pose.position.x; // <- SLAM 위치로 맞추기!
        position_y_ = msg->pose.pose.position.y; // <- SLAM 위치로 맞추기!
        slam_init_theta_ = yaw;
        slam_heading_initialized_ = true;
        RCLCPP_INFO(node_->get_logger(), "[TracerMessenger] theta_ initialized from SLAM odom: %.3f", yaw);
      }
    }

    void PublishStateToROS()
    {
      current_time_ = node_->get_clock()->now();

      static bool init_run = true;
      if (init_run)
      {
        last_time_ = current_time_;
        init_run = false;
        return;
      }
      double dt = (current_time_ - last_time_).seconds();

      auto state = tracer_->GetRobotState();

      // publish tracer state message
      tracer_msgs::msg::TracerStatus status_msg;
      tracer_msgs::msg::TracerRCState rc_state_msg;
      status_msg.header.stamp = current_time_;

      status_msg.linear_velocity = state.motion_state.linear_velocity;
      status_msg.angular_velocity = state.motion_state.angular_velocity;

      // status_msg.vehicle_state = state.system_state.vehicle_state;
      status_msg.control_mode = state.system_state.control_mode;
      status_msg.error_code = state.system_state.error_code;
      status_msg.battery_voltage = state.system_state.battery_voltage;

      rc_state_msg.stick_left_h = state.rc_state.stick_left_h;
      rc_state_msg.stick_left_v = state.rc_state.stick_left_v;
      rc_state_msg.stick_right_h = state.rc_state.stick_right_h;
      rc_state_msg.stick_right_v = state.rc_state.stick_right_v;

      rc_state_msg.swa = state.rc_state.swa;
      rc_state_msg.swb = state.rc_state.swb;
      rc_state_msg.swc = state.rc_state.swc;
      rc_state_msg.swd = state.rc_state.swd;

      rc_state_msg.var_a = state.rc_state.var_a;

      rc_status_pub_->publish(rc_state_msg);

      auto actuator = tracer_->GetActuatorState();

      for (int i = 0; i < 2; ++i)
      {
        // actuator_hs_state
        uint8_t motor_id = actuator.actuator_hs_state[i].motor_id;

        status_msg.actuator_states[motor_id].rpm =
            actuator.actuator_hs_state[i].rpm;
        status_msg.actuator_states[motor_id].current =
            actuator.actuator_hs_state[i].current;
        // status_msg.actuator_states[motor_id].pulse_count =
        // actuator.actuator_hs_state[i].pulse_count;

        // actuator_ls_state
        // motor_id = actuator.actuator_ls_state[i].motor_id;

        // status_msg.actuator_states[motor_id].driver_voltage =
        // actuator.actuator_ls_state[i].driver_voltage;
        // status_msg.actuator_states[motor_id].driver_temperature =
        // actuator.actuator_ls_state[i].driver_temp;
        // status_msg.actuator_states[motor_id].motor_temperature =
        // actuator.actuator_ls_state[i].motor_temp;
        // status_msg.actuator_states[motor_id].driver_state =
        // actuator.actuator_ls_state[i].driver_state;
      }

      // status_msg.light_control_enabled = state.light_state.enable_cmd_ctrl;
      // status_msg.front_light_state.mode = state.light_state.front_light.mode;
      // status_msg.front_light_state.custom_value =
      // state.light_state.front_light.custom_value;
      // status_msg.rear_light_state.mode = state.light_state.rear_light.mode;
      // status_msg.rear_light_state.custom_value =
      //     state.light_state.rear_light.custom_value;
      status_pub_->publish(status_msg);

      // publish odometry and tf
      PublishOdometryToROS(state.motion_state, dt);

      // record time for next integration
      last_time_ = current_time_;
    }

  private:
    std::shared_ptr<TracerType> tracer_;
    rclcpp::Node *node_;

    std::string odom_frame_;
    std::string base_frame_;
    std::string odom_topic_name_;

    bool simulated_robot_ = false;
    int sim_control_rate_ = 50;
    bool slam_heading_initialized_ = true;
    double slam_init_theta_ = 0.0;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr slam_odom_sub_;

    std::mutex twist_mutex_;
    geometry_msgs::msg::Twist current_twist_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<tracer_msgs::msg::TracerStatus>::SharedPtr status_pub_;
    rclcpp::Publisher<tracer_msgs::msg::TracerRCState>::SharedPtr rc_status_pub_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr motion_cmd_sub_;
    rclcpp::Subscription<tracer_msgs::msg::TracerLightCmd>::SharedPtr
        light_cmd_sub_;

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // speed variables
    double position_x_ = 0.0;
    double position_y_ = 0.0;
    double theta_ = 0.0;

    rclcpp::Time last_time_;
    rclcpp::Time current_time_;

    void TwistCmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
      if (!simulated_robot_)
      {
        SetTracerMotionCommand(msg);
      }
      else
      {
        std::lock_guard<std::mutex> guard(twist_mutex_);
        current_twist_ = *msg.get();
      }
      // ROS_INFO("Cmd received:%f, %f", msg->linear.x, msg->angular.z);
    }
    void SetTracerMotionCommand(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
      tracer_->SetMotionCommand(msg->linear.x, msg->angular.z);
    }

    void LightCmdCallback(const tracer_msgs::msg::TracerLightCmd::SharedPtr msg)
    {
      if (!simulated_robot_)
      {
        if (msg->cmd_ctrl_allowed)
        {
          LightCommandMessage cmd;

          switch (msg->front_mode)
          {
          case tracer_msgs::msg::TracerLightCmd::LIGHT_CONST_OFF:
          {
            cmd.front_light.mode = CONST_OFF;
            break;
          }
          case tracer_msgs::msg::TracerLightCmd::LIGHT_CONST_ON:
          {
            cmd.front_light.mode = CONST_ON;
            break;
          }
          case tracer_msgs::msg::TracerLightCmd::LIGHT_BREATH:
          {
            cmd.front_light.mode = BREATH;
            break;
          }
          case tracer_msgs::msg::TracerLightCmd::LIGHT_CUSTOM:
          {
            cmd.front_light.mode = CUSTOM;
            cmd.front_light.custom_value = msg->front_custom_value;
            break;
          }
          }

          // switch (msg->rear_mode)
          // {
          //   case tracer_msgs::TracerLightCmd::LIGHT_CONST_OFF:
          //   {
          //       cmd.rear_light.mode = CONST_OFF;
          //       break;
          //   }
          //   case tracer_msgs::TracerLightCmd::LIGHT_CONST_ON:
          //   {
          //       cmd.rear_light.mode = CONST_ON;
          //       break;
          //   }
          //   case tracer_msgs::TracerLightCmd::LIGHT_BREATH:
          //   {
          //       cmd.rear_light.mode = BREATH;
          //       break;
          //   }
          //   case tracer_msgs::TracerLightCmd::LIGHT_CUSTOM:
          //   {
          //       cmd.rear_light.mode = CUSTOM;
          //       cmd.rear_light.custom_value = msg->rear_custom_value;
          //       break;
          //   }
          // }
          tracer_->SetLightCommand(cmd.front_light.mode, cmd.front_light.custom_value);
        }
      }
      else
      {
        std::cout << "simulated robot received light control cmd" << std::endl;
      }
    }

    geometry_msgs::msg::Quaternion createQuaternionMsgFromYaw(double yaw)
    {
      tf2::Quaternion q;
      q.setRPY(0, 0, yaw);
      return tf2::toMsg(q);
    }

    void PublishOdometryToROS(const MotionStateMessage &msg, double dt)
    {
      // perform numerical integration to get an estimation of pose
      double linear_speed = msg.linear_velocity;
      double angular_speed = msg.angular_velocity;

      // if (std::is_base_of<TracerMiniOmniRobot, TracerType>::value) {
      //   lateral_speed = msg.lateral_velocity;
      // } else {
      //   lateral_speed = 0;
      // }

      double d_x = linear_speed * std::cos(theta_) * dt;
      double d_y = linear_speed * std::sin(theta_) * dt;
      double d_theta = angular_speed * dt;

      // RCLCPP_INFO(node_->get_logger(),
      //             "[ODOM] dt: %.3f, theta_: %.3f, d_x: %.3f, d_y: %.3f, linear_speed: %.3f, angular_speed: %.3f",
      //             dt, theta_, d_x, d_y, linear_speed, angular_speed);

      if (!slam_heading_initialized_)
      {
        // SLAM 초기 yaw 동기화 전까지는 오도메트리 누적 계산 보류
        return;
      }

      position_x_ += d_x;
      position_y_ += d_y;
      theta_ += d_theta;

      geometry_msgs::msg::Quaternion odom_quat =
          createQuaternionMsgFromYaw(theta_);

      // publish tf transformation
      geometry_msgs::msg::TransformStamped tf_msg;
      tf_msg.header.stamp = current_time_;
      tf_msg.header.frame_id = odom_frame_;
      tf_msg.child_frame_id = base_frame_;

      tf_msg.transform.translation.x = position_x_;
      tf_msg.transform.translation.y = position_y_;
      tf_msg.transform.translation.z = 0.0;
      tf_msg.transform.rotation = odom_quat;

      tf_broadcaster_->sendTransform(tf_msg);

      // publish odometry and tf messages
      nav_msgs::msg::Odometry odom_msg;
      odom_msg.header.stamp = current_time_;
      odom_msg.header.frame_id = odom_frame_;
      odom_msg.child_frame_id = base_frame_;

      odom_msg.pose.pose.position.x = position_x_;
      odom_msg.pose.pose.position.y = position_y_;
      odom_msg.pose.pose.position.z = 0.0;
      odom_msg.pose.pose.orientation = odom_quat;

      odom_msg.twist.twist.linear.x = linear_speed;
      odom_msg.twist.twist.linear.y = 0.0;
      odom_msg.twist.twist.angular.z = angular_speed;

      odom_pub_->publish(odom_msg);
    }
  };
} // namespace westonrobot

#endif /* SCOUT_MESSENGER_HPP */


#include <memory>
#include <algorithm>
#include <thread>
#include <cmath>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "mecanum_wheel_controller/ddsm_ctrl.hpp"

class MecanumWheelControllerNode : public rclcpp::Node
{
public:
  MecanumWheelControllerNode() : Node("mecanum_wheel_controller_node")
  {
    // Create subscription to cmd_vel topic
    cmd_vel_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, std::bind(&MecanumWheelControllerNode::cmd_vel_callback, this, std::placeholders::_1));
    
    // Parameters for mecanum wheel configuration
    this->declare_parameter("wheel_radius", 0.05);      // Wheel radius in meters
    this->declare_parameter("wheel_base_x", 0.3);       // Distance between front and rear wheels
    this->declare_parameter("wheel_base_y", 0.3);       // Distance between left and right wheels
    this->declare_parameter("serial_port", "/dev/ttyACM0");  // Serial port for DDSM motors
    this->declare_parameter("baud_rate", 115200);       // Baud rate for serial communication
    //　モーターのIDは事前に設定する必要あり
    this->declare_parameter("motor_ids", std::vector<int64_t>{1, 2, 3, 4});  // Motor IDs [FL, FR, BL, BR]
    
    // Initialize DDSM controller (single controller for all motors on same serial bus)
    ddsm_controller_ = std::make_unique<DDSM_CTRL>();
    
    // Initialize serial connection and set motor type
    std::string port = this->get_parameter("serial_port").as_string();
    int baud = this->get_parameter("baud_rate").as_int();
    
    bool init_success = false;
    if (ddsm_controller_->init(port, baud)) {
      // Set motor type to DDSM115
      if (ddsm_controller_->set_ddsm_type(115) == TYPE_DDSM115) {
        RCLCPP_INFO(this->get_logger(), "DDSM115 motor type set successfully");
        
        // Set all motors to velocity/speed mode (mode 2)
        auto motor_ids = this->get_parameter("motor_ids").as_integer_array();
        init_success = true;
        
        for (size_t i = 0; i < motor_ids.size() && i < 4; i++) {
          RCLCPP_INFO(this->get_logger(), "Setting motor ID %ld to velocity mode", motor_ids[i]);
          ddsm_controller_->ddsm_change_mode(motor_ids[i], DDSM115_SPEED_MODE);
          std::this_thread::sleep_for(std::chrono::milliseconds(50)); // Delay between mode changes
        }
        
        // Clear any residual data in the buffer after mode changes
        ddsm_controller_->clear_ddsm_buffer();
        
        RCLCPP_INFO(this->get_logger(), "All motors set to velocity mode (RPM control)");
      } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to set motor type to DDSM115");
      }
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to initialize DDSM controller on port %s", port.c_str());
    }
    
    if (init_success) {
      RCLCPP_INFO(this->get_logger(), "Mecanum wheel controller node started successfully!");
    } else {
      RCLCPP_WARN(this->get_logger(), "Mecanum wheel controller node started with some initialization errors");
    }
    
    RCLCPP_INFO(this->get_logger(), "Waiting for cmd_vel messages...");
  }
  
  ~MecanumWheelControllerNode() {
    // Stop all motors on shutdown
    stop_all_motors();
  }

private:
  // Member variables
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;
  std::unique_ptr<DDSM_CTRL> ddsm_controller_; // Single controller for all motors on same serial bus

  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    // Get parameters
    double wheel_radius = this->get_parameter("wheel_radius").as_double();
    double wheel_base_x = this->get_parameter("wheel_base_x").as_double();
    double wheel_base_y = this->get_parameter("wheel_base_y").as_double();
    
    // Extract velocity components
    double vx = msg->linear.x;   // Forward/backward velocity
    double vy = msg->linear.y;   // Left/right strafe velocity
    double wz = msg->angular.z;  // Rotational velocity
    
    // Mecanum wheel kinematics
    // For a mecanum wheel robot with wheels arranged as:
    //   FL(+)  FR(-)
    //   BL(-)  BR(+)
    // Where (+) means forward rolling gives positive robot motion
    
    double lx = wheel_base_x / 2.0;  // Half distance between front and rear
    double ly = wheel_base_y / 2.0;  // Half distance between left and right
    
    // Calculate individual wheel velocities (rad/s)
    double front_left_vel  = (vx - vy - (lx + ly) * wz) / wheel_radius;
    double front_right_vel = (vx + vy + (lx + ly) * wz) / wheel_radius;
    double back_left_vel   = (vx + vy - (lx + ly) * wz) / wheel_radius;
    double back_right_vel  = (vx - vy + (lx + ly) * wz) / wheel_radius;
    
    // Log the received cmd_vel and calculated wheel velocities
    RCLCPP_INFO(this->get_logger(), 
                "Received cmd_vel: vx=%.3f, vy=%.3f, wz=%.3f", 
                vx, vy, wz);
    
    RCLCPP_INFO(this->get_logger(), 
                "Wheel velocities (rad/s): FL=%.3f, FR=%.3f, BL=%.3f, BR=%.3f",
                front_left_vel, front_right_vel, back_left_vel, back_right_vel);
    
    // Send commands to DDSM motors
    send_motor_commands(front_left_vel, front_right_vel, back_left_vel, back_right_vel);
  }

  void send_motor_commands(double fl, double fr, double bl, double br) {
    auto motor_ids = this->get_parameter("motor_ids").as_integer_array();
    
    // Convert rad/s to RPM for DDSM115 motors
    // DDSM115 speed range: -330 to +330 rpm
    const double rad_s_to_rpm = 60.0 / (2.0 * M_PI); // Convert rad/s to RPM (≈9.549)
    
    // Convert wheel velocities to RPM
    int fl_rpm = static_cast<int>(fl * rad_s_to_rpm);
    int fr_rpm = static_cast<int>(fr * rad_s_to_rpm);
    int bl_rpm = static_cast<int>(bl * rad_s_to_rpm);
    int br_rpm = static_cast<int>(br * rad_s_to_rpm);
    
    // Clamp commands to DDSM115 limits (-330 to 330 rpm)
    fl_rpm = std::max(-330, std::min(330, fl_rpm));
    fr_rpm = std::max(-330, std::min(330, fr_rpm));
    bl_rpm = std::max(-330, std::min(330, bl_rpm));
    br_rpm = std::max(-330, std::min(330, br_rpm));
    
    RCLCPP_INFO(this->get_logger(), 
                "Motor RPM commands: FL=%d, FR=%d, BL=%d, BR=%d",
                fl_rpm, fr_rpm, bl_rpm, br_rpm);
    
    try {
      // Send velocity commands to each motor in RPM
      // acceleration_time: 1 = 0.1ms per rpm (fast acceleration)
      if (motor_ids.size() >= 4) {
        ddsm_controller_->ddsm_ctrl(motor_ids[0], fl_rpm, 1); // Front Left
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        
        ddsm_controller_->ddsm_ctrl(motor_ids[1], fr_rpm, 1); // Front Right  
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        
        ddsm_controller_->ddsm_ctrl(motor_ids[2], bl_rpm, 1); // Back Left
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        
        ddsm_controller_->ddsm_ctrl(motor_ids[3], br_rpm, 1); // Back Right
      }
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Error sending motor commands: %s", e.what());
    }
  }
  
  void stop_all_motors() {
    auto motor_ids = this->get_parameter("motor_ids").as_integer_array();
    
    try {
      for (size_t i = 0; i < std::min(motor_ids.size(), static_cast<size_t>(4)); i++) {
        ddsm_controller_->ddsm_stop(motor_ids[i]);
        std::this_thread::sleep_for(std::chrono::milliseconds(5)); // Small delay between stop commands
      }
      RCLCPP_INFO(this->get_logger(), "All motors stopped");
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Error stopping motors: %s", e.what());
    }
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MecanumWheelControllerNode>());
  rclcpp::shutdown();
  return 0;
}

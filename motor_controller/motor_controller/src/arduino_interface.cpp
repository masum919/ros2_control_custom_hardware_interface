#include "motor_controller/arduino_interface.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <iostream>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include <string>
#include <std_msgs/msg/int32_multi_array.hpp>  // Include the proper message type
#include <cmath>






namespace arduino_controller
{
ArduinoInterface::ArduinoInterface() 
{
}



ArduinoInterface::~ArduinoInterface()
{
    if (SerialPort != -1)
    {
        close(SerialPort);
    }
}


CallbackReturn ArduinoInterface::on_init(const hardware_interface::HardwareInfo &hardware_info)
{
  CallbackReturn result = hardware_interface::SystemInterface::on_init(hardware_info);
  if (result != CallbackReturn::SUCCESS)
  {
    return result;
  }

// Declare the node_ variable
//rclcpp::Node::SharedPtr node_;

// Initialize the publisher here
// node_ = std::make_shared<rclcpp::Node>("arduino_controller_node");
// publisher_ = node_->create_publisher<std_msgs::msg::Int32MultiArray>("arduino_data", 10);


  try
  {
  

    std::string port = "/dev/ttyACM0";
    SerialPort = open(port.c_str(), O_RDWR);
    if (SerialPort < 0)
    {

        RCLCPP_WARN(rclcpp::get_logger("arduino_controller_interface"), 
                    "Unable to open serial port %s. Error: %s", port.c_str(), strerror(errno));
        RCLCPP_WARN(rclcpp::get_logger("arduino_controller_interface"), 
                    "Controller will run in simulation mode.");
        return CallbackReturn::SUCCESS;  // Continue even if Arduino is not connected
    }

    if (tcgetattr(SerialPort, &tty) != 0)
    {
        RCLCPP_ERROR(rclcpp::get_logger("arduino_controller_interface"), "Error %i from tcgetattr: %s", errno, strerror(errno));
        close(SerialPort);
        return CallbackReturn::ERROR;
    }

    tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
    tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
    tty.c_cflag &= ~CSIZE; // Clear all bits that set the data size
    tty.c_cflag |= CS8; // 8 bits per byte (most common)
    tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
    tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO; // Disable echo
    tty.c_lflag &= ~ECHOE; // Disable erasure
    tty.c_lflag &= ~ECHONL; // Disable new-line echo
    tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);

    tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
    tty.c_oflag &= ~ONLCR;

    tty.c_cc[VTIME] = 1;
    tty.c_cc[VMIN] = 0;

    speed_t speed = B115200;
    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);

    tcflush(SerialPort, TCIFLUSH);
    if (tcsetattr(SerialPort, TCSANOW, &tty) != 0)
    {
        RCLCPP_ERROR(rclcpp::get_logger("CustomHardware"), "Error %i from tcsetattr: %s\n", errno, strerror(errno));
        return hardware_interface::CallbackReturn::ERROR;
    }
    RCLCPP_INFO(rclcpp::get_logger("CustomHardware"), "SERIAL PORT OPENED: %d! WAITING...", SerialPort);

    auto t_start = std::chrono::high_resolution_clock::now();
    while(true)
    {
        auto t_end = std::chrono::high_resolution_clock::now();
        double elapsed_time_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();
        if(elapsed_time_ms > 3000)
        {
            break;
        }
    }


   
  }


  catch(std::exception &e)
  {

    RCLCPP_WARN(
      rclcpp::get_logger("arduino_actuator_interface"),
      "Error during initialization: %s. Running in simulation mode.", e.what()
    );
    return CallbackReturn::SUCCESS;  // Continue even if there's an error
  }

  velocity_commands_.reserve(info_.joints.size());
  velocity_states_.reserve(info_.joints.size());
  prev_velocity_commands_.reserve(info_.joints.size());
  position_states_.reserve(info_.joints.size());

  return CallbackReturn::SUCCESS;
}


std::vector<hardware_interface::StateInterface> ArduinoInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;


  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_states_[i]));
  }

  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_states_[i]));
  }

  return state_interfaces;
}


std::vector<hardware_interface::CommandInterface> ArduinoInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_commands_[i]));
  }

  return command_interfaces;
}


CallbackReturn ArduinoInterface::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("ArduinoInterface"), "Starting robot hardware ...");

  // Reset commands and states
  velocity_commands_ = { 0.0, 0.0};
  prev_velocity_commands_ = { 0.0, 0.0};
  velocity_states_ = { 0.0, 0.0};
  position_states_ = { 0.0, 0.0};



  RCLCPP_INFO(rclcpp::get_logger("ArduinoInterface"),
              "Hardware started, ready to take commands");
  return CallbackReturn::SUCCESS;
}


CallbackReturn ArduinoInterface::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
    if(SerialPort == -1)
    {
        return hardware_interface::CallbackReturn::SUCCESS;
    }
    tcflush(SerialPort, TCIFLUSH);
    close(SerialPort);
    return hardware_interface::CallbackReturn::SUCCESS;
}


int ArduinoInterface::WriteToSerial(const unsigned char* buf, int nBytes)
{
    return ::write(SerialPort, const_cast<unsigned char*>(buf), nBytes);
}

int ArduinoInterface::ReadSerial(unsigned char* buf, int nBytes)
{
    auto t_start = std::chrono::high_resolution_clock::now();
    int n = 0;
    while(n < nBytes)
    {
        int ret = ::read(SerialPort, &buf[n], 1);
        if(ret < 0)
        {
            return ret;
        }

        n+=ret;
        auto t_end = std::chrono::high_resolution_clock::now();
        double elapsed_time_ms = std::chrono::duration<double, std::milli>(t_end-t_start).count();
        if(elapsed_time_ms > 10000)
        {
            break;
        }
    }
    return n;
}



// This function is responsible for writing commands to the Arduino
hardware_interface::return_type ArduinoInterface::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
  RCLCPP_INFO(rclcpp::get_logger("arduino_actuator_interface"), "Writing");

  try {
    // Get the left and right velocity commands
    float rpmValue1 = static_cast<float>(velocity_commands_.at(0));
    int dirValue1 = (rpmValue1 >= 0) ? 0 : 1;  // 0 for forward, 1 for reverse

    float rpmValue2 = static_cast<float>(velocity_commands_.at(1));
    int dirValue2 = (rpmValue2 >= 0) ? 0 : 1;  // 0 for forward, 1 for reverse

    // Create a string with the command data
    std::string data = std::to_string(rpmValue1) + " " + std::to_string(dirValue1) + " " +
                       std::to_string(rpmValue2) + " " + std::to_string(dirValue2) + "\n";

    // Write the command data to the serial port
    WriteToSerial(reinterpret_cast<const unsigned char*>(data.c_str()), data.length());
    RCLCPP_INFO(rclcpp::get_logger("arduino_actuator_interface"), "Writing %s", data.c_str());

    // Throttle the data transfer to avoid overwhelming the Arduino
    std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Adjust the sleep duration as needed
  }
  catch (const std::exception& e) {
    // Handle any exceptions that occur during the write process
    RCLCPP_FATAL(rclcpp::get_logger("arduino_actuator_interface"), "Error: %s", e.what());
    return hardware_interface::return_type::ERROR;
  }

  // Log the left and right joint commands for debugging purposes
  RCLCPP_INFO(rclcpp::get_logger("arduino_actuator_interface"), "Left joint command: %.2f, Direction: %d", velocity_commands_.at(0), (velocity_commands_.at(0) >= 0) ? 0 : 1);
  RCLCPP_INFO(rclcpp::get_logger("arduino_actuator_interface"), "Right joint command: %.2f, Direction: %d", velocity_commands_.at(1), (velocity_commands_.at(1) >= 0) ? 0 : 1);

  return hardware_interface::return_type::OK;
}



// This function is responsible for reading data from the Arduino
hardware_interface::return_type ArduinoInterface::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
    // Convert the period to seconds
    double dt = period.seconds();

    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
        // Update velocity states with the commanded velocities
        velocity_states_[i] = velocity_commands_[i];

        // Calculate position change based on velocity and time
        double delta_position = velocity_states_[i] * dt;

        // Update position states
        position_states_[i] += delta_position;

        // Normalize position to keep it within [-pi, pi] range
        while (position_states_[i] > M_PI) position_states_[i] -= 2 * M_PI;
        while (position_states_[i] < -M_PI) position_states_[i] += 2 * M_PI;
    }

    RCLCPP_INFO(rclcpp::get_logger("arduino_actuator_interface"), 
                "Joint 1 position: %.2f, velocity: %.2f", 
                position_states_[0], velocity_states_[0]);
    RCLCPP_INFO(rclcpp::get_logger("arduino_actuator_interface"), 
                "Joint 2 position: %.2f, velocity: %.2f", 
                position_states_[1], velocity_states_[1]);

    return hardware_interface::return_type::OK;
}




// rclcpp::Node::SharedPtr node_;  // Node for publisher
// rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr publisher_;  // Publisher declaration



}  // namespace arduinobot_controller

PLUGINLIB_EXPORT_CLASS(arduino_controller::ArduinoInterface, hardware_interface::SystemInterface)

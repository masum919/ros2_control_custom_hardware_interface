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
hardware_interface::return_type ArduinoInterface::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  RCLCPP_INFO(rclcpp::get_logger("arduino_actuator_interface"), "Writing");

  try
  {
    // Get the left and right velocity commands
    int left = static_cast<int>(velocity_commands_.at(0));
    int right = static_cast<int>(velocity_commands_.at(1));

    // Create a string with the command data
    std::string data = std::to_string(abs(left)) + ";" + (left >= 0 ? "1" : "0") + ";" +
                       std::to_string(abs(right)) + ";" + (right >= 0 ? "1" : "0") + "\n";

    // Write the command data to the serial port
    WriteToSerial(reinterpret_cast<const unsigned char*>(data.c_str()), data.length());
    RCLCPP_INFO(rclcpp::get_logger("arduino_actuator_interface"), "Writing %s", data.c_str());

    // Throttle the data transfer to avoid overwhelming the Arduino
    std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Adjust the sleep duration as needed
  }
  catch (const std::exception& e)
  {
    // Handle any exceptions that occur during the write process
    RCLCPP_FATAL(rclcpp::get_logger("arduino_actuator_interface"), "Error: %s", e.what());
    return hardware_interface::return_type::ERROR;
  }

  // Log the left and right joint commands for debugging purposes
  RCLCPP_INFO(rclcpp::get_logger("arduino_actuator_interface"), "Left joint command: %d", static_cast<int>(velocity_commands_.at(0)));
  RCLCPP_INFO(rclcpp::get_logger("arduino_actuator_interface"), "Right joint command: %d", static_cast<int>(velocity_commands_.at(1)));

  return hardware_interface::return_type::OK;
}

// This function is responsible for reading data from the Arduino
hardware_interface::return_type ArduinoInterface::read(const rclcpp::Time & /*time*/,
                             const rclcpp::Duration & /*period*/)
{
  unsigned char buf[10] = {0};  // Buffer to store the received data
  int bytesRead = ReadSerial(buf, 10);  // Read data from the serial port

  if (bytesRead == 10) {
  int32_t encoderValue, rpm2Value;
  int8_t dir1Value, dir2Value;
  
  // Extract the received data from the buffer
  memcpy(&encoderValue, buf, 4);
  dir1Value = buf[4];
  memcpy(&rpm2Value, buf + 5, 4);
  dir2Value = buf[9];

  // Log the received data for debugging purposes
  RCLCPP_INFO(rclcpp::get_logger("arduino_actuator_interface"), 
        "Received from Arduino: Encoder=%d, Dir1=%d, RPM2=%d, Dir2=%d",
        encoderValue, dir1Value, rpm2Value, dir2Value);

  } else {
  // Handle the case when incomplete or no data is received from Arduino
  RCLCPP_WARN(rclcpp::get_logger("arduino_actuator_interface"), 
        "Incomplete or no data received from Arduino. Bytes read: %d", bytesRead);
  }

  return hardware_interface::return_type::OK;
}








}  // namespace arduinobot_controller

PLUGINLIB_EXPORT_CLASS(arduino_controller::ArduinoInterface, hardware_interface::SystemInterface)
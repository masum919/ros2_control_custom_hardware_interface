# ros2_control_custom_hardware_interface
A ros2 control implementation of a custom hardware interface (arduino) to send motor commands.
This implementation sends commands to two motors (can be extended for many motors) and reads encoder values from the motors. The arduino code handles only one motor and can be easily extended for more motors.

# Some important commands
Launch the controller with the command `ros2 launch motor_controller motor.launch.py`
Send a sample motor command from a separate terminal `ros2 topic pub -r 10 /motor_controller/commands std_msgs/msg/Float64MultiArray "data: [861.0, 9892.0]"`
List out all the active controllers with the command `ros2 control list_controllers`
Deactivate the motor controller with the command `ros2 control set_controller_state motor_controller inactive`
Unload the motor controller with the command `ros2 control unload_controller motor_controller`
Reload the controller with the command `ros2 control load_controller motor_controller`
Reactivate the controller with the command `ros2 control set_controller_state motor_controller_active`

If you intend to use multiple controllers for controlling multiple Arduinos from the same machine make the following changes in your config.yaml and your launch files for all the controllers.

![config_yaml](https://github.com/user-attachments/assets/b8f0a988-de61-49e6-b12d-c4d073f95306)
![launch_py](https://github.com/user-attachments/assets/5338a053-d0fd-466f-93b4-ff59dfff0a52)


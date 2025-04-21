# FRA532_LAB1

เดี่ยวมาเขียนต่อ

## Usage

Run simulation it's has two mode on steering control mode
1. basic
2. nscc (Default)

```bash
# Run the project
ros2 launch robot_bringup robot_bringup.launch.py steering_mode:=nscc
```

Control with Keyboard

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Controll with Controller Mode
1. pure_pursuit (Default)
2. pid
3. stanley 

```bash
ros2 launch robot_controller robot_controller.launch.py control_mode:=pure_pursuit
```

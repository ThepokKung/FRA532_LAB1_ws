# FRA532_LAB1

เดี่ยวมาเขียนต่อ

## Usage

Run simulation

```bash
# Run the project
ros2 launch robot_sim sim.launch.py
```

Run Invert Model

```bash
# Select Model Basic or NSCC
ros2 run robot_controller InverKinematic-basicmodel.py
# or
ros2 run robot_controller InverKinematic-nscc.py
```

Control with Keyboard

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
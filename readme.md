# FRA532_LAB1

Update 14/4/24

อัพเดท To Jazzy เพราะอัพเดท Package แล้วแตก เลยย้ายแม่ง 

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
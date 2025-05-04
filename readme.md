# FRA532_LAB1

To be updated soon.

Pending tasks:
* Implement Extended Kalman Filter (EKF)

## Usage

Run simulation it's has two mode on steering control mode
1. basic
2. nscc (Default)

example :
```bash
# Run the project
ros2 launch robot_bringup robot_bringup.launch.py steering_mode:=nscc

#can change steering_mode to nscc or basic
```

Control with Keyboard

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Controll with Controller Mode
1. pure_pursuit (Default)
2. pid
3. stanley 

example :
```bash
# Run controller
ros2 launch robot_controller robot_controller.launch.py control_mode:=pure_pursuit
#can change control_mode to pure_pursuit, pid or stanley 
```
> [!WARNING]
> When you need to restart the controller, please restart the simulation.
> Please restart simulation

## LAB 1.1 Mobile Robot Kinematics
### 1. Model Ackerman Steering

A 4-wheel mobile robot with Ackerman steering has been created using the Limo robot as a model in the package [`robot_description`](/src/robot_description/), and the TF of the robot is shown below:

<!-- ![Fake_limo_tf.png](/images/fake_limo_tf.png) -->
<img src='./images/fake_limo_tf.png' alt='Fake limo TF' class="image-full-width" >

The robot has also been imported into Gazebo in the package `robot_sim`.

<!-- ![robotingazebo.png](/images/robotingazebo.png) -->
<img src='./images/robotingazebo.png' alt='robot in Gazebo world' class="image-full-width">

### 2. Inverse Kinematics
Inverse Kinematics has two models: `Basic Model` and `No Slip Condition Constraints`, implemented in the nodes [`InverKinematic-basic.py`](/src/robot_controller/scripts/Kinematic/InverKinematic-basic.py) and [`InverKinematic-nscc.py`](/src/robot_controller/scripts/Kinematic/InverKinematic-nscc.py).

#### 2.1 Basic Model
The Basic Model assumes that the steering angles of both wheels are equal, similar to a bicycle.

The steering angle in the Basic Model is calculated as follows:

```math
\delta = \arctan\left(\frac{\omega \cdot L}{V_x}\right)
```
Where:
* $\delta$ is the steering angle
* $\omega$ is the angular velocity
* $L$ is the wheelbase length
* $V_x$ is the forward velocity


#### 2.2 No Slip condition constraints
The No Slip Condition Constraints model ensures that the steering angles of the two wheels are different, which helps prevent the vehicle from skidding.


```math
\delta = \arctan\left(\frac{\omega \cdot L}{V_x}\right)
```
จากนั้นคำนวณมุมเลี้ยวซ้าย-ขวาตามหลัก Ackermann steering geometry:

```math
\delta_{left} = \arctan\left(\frac{L \cdot \tan(\delta)}{L + 0.5 \cdot W \cdot \tan(\delta)}\right)
```
```math
\delta_{right} = \arctan\left(\frac{L \cdot \tan(\delta)}{L - 0.5 \cdot W \cdot \tan(\delta)}\right)
```

Where:
* $\delta$ is the center steering angle
* $\delta_{left}$ is the left wheel steering angle
* $\delta_{right}$ is the right wheel steering angle
* $L$ is the wheelbase length (distance between front and rear axles)
* $W$ is the track width (distance between left and right wheels)
* $\omega$ is the angular velocity
* $V_x$ is the forward velocity

### 3. Forward Kinematics

Forward Kinematics is used to predict the robot's future position and orientation based on its current state and motion parameters. This is crucial for simulating and controlling the robot's movement in various scenarios.

Forward Kinematics has 3 Models: `YawRate`,`Single-Track` and `Double-Track`, all implemented in the node: [`ForwardKinematic-All.py`](/src/robot_controller/scripts/Kinematic/ForwardKinematic-All.py)

#### 3.1 Yaw Rate Model 
For the Yaw Rate model, we use the IMU's yaw rate and wheel speeds:

```math
V = \frac{v_{rear\_left} + v_{rear\_right}}{2}
```
```math
\omega = \omega_{IMU}
```
Then update position with midpoint integration:

```math
\theta_{mid} = \theta_t + 0.5 \cdot \omega \cdot dt
```
```math
x_{t+1} = x_t + V \cdot \cos(\theta_{mid}) \cdot dt
```
```math
y_{t+1} = y_t + V \cdot \sin(\theta_{mid}) \cdot dt
```
```math
\theta_{t+1} = \theta_t + \omega \cdot dt
```
Where:

* $V$ is the linear velocity of the robot
* $\omega$ is the angular velocity of the robot
* $dt$ is the time step
* $(x, y, \theta)$ is the robot's pose (position and orientation)
* $dt$ is the time step
* $\theta$ is the robot's orientation (measured in radians)
  
#### 3.2 Single-track model
The Single-Track model, also known as the Bicycle model, simplifies the robot's dynamics by treating it as a single-track vehicle. Unlike the Yaw Rate model, it incorporates the average steering angle of the front wheels to calculate angular velocity.

For the Single-Track (Bicycle) model:


```math
V = \frac{v_{rear\_left} + v_{rear\_right}}{2}
```
```math
\delta = \frac{\delta_{left} + \delta_{right}}{2}
```
```math
\omega = \frac{V}{L} \cdot \tan(\delta)
```

Where $\delta$ is the average steering angle of both front wheels.

Update position with midpoint integration

```math
\theta_{mid} = \theta_t + 0.5 \cdot \omega \cdot dt
```
```math
x_{t+1} = x_t + V \cdot \cos(\theta_{mid}) \cdot dt
```
```math
y_{t+1} = y_t + V \cdot \sin(\theta_{mid}) \cdot dt
```
```math
\theta_{t+1} = \theta_t + \omega \cdot dt
```

Where:

* $V$ and $\omega$ are defined in the consolidated section above.
* $v_{rear_left}$ and $v_{rear_right}$ are the rear wheel velocities
* $L$ is the wheelbase length (distance between front and rear axles)
#### 3.3 Double-track model
The Double-Track model provides a more detailed representation of the robot's motion by considering the individual contributions of each wheel, unlike the Single-Track model which averages the steering angles.

For the Double-Track model (differential approximation):
#### 3.3 Double-track model
For the Double-Track model (differential approximation):

The angular velocity $\omega$ in this model is derived from the difference in velocities of the rear wheels. This approach captures the rotational dynamics of the robot, as the difference in wheel speeds directly influences the robot's turning rate. The track width $W$ is used to normalize this difference, ensuring the calculation aligns with the robot's physical dimensions.

```math
V = \frac{v_{rear\_left} + v_{rear\_right}}{2}
```

```math
\omega = \frac{v_{rear\_right} - v_{rear\_left}}{W}
```
Where $W$ is the track width.

Update position with midpoint integration:

```math
\theta_{mid} = \theta_t + 0.5 \cdot \omega \cdot dt
```
```math
x_{t+1} = x_t + V \cdot \cos(\theta_{mid}) \cdot dt
```
```math
y_{t+1} = y_t + V \cdot \sin(\theta_{mid}) \cdot dt
```
```math
\theta_{t+1} = \theta_t + \omega \cdot dt
```


* $V$ and $\omega$ are defined in the consolidated section above.
* $v_{rear_left}$ and $v_{rear_right}$ are the rear wheel velocities
* $L$ is the wheelbase length
* $W$ is the track width
* $\theta$ is the robot's orientation

2. Single-Track model: Uses average steering angle and bicycle model
3. Double-Track model: Uses differential wheel speeds (similar to tank-drive or differential drive)
This updated version accurately reflects the algorithms implemented in your `ForwardKinematic-All.py` file, with each model using a different approach to calculate angular velocity (ω):

1. Yaw Rate model: Uses IMU's yaw rate directly.
2. Single-Track model: Uses average steering angle and bicycle model.
3. Double-Track model: Uses differential wheel speeds (similar to tank-drive or differential drive).

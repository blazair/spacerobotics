# Combined Cylinder Estimation and IBVS Marker Landing

This mission combines **autonomous cylinder estimation** during a circular flight and a **visual servoing (IBVS)** landing on one of two ArUco-marked cylinders.

---

## Launch the Mission

Load the sim environment
```
ros2 launch mission cylinder_landing.launch.py
```
Run the aruco detection marker (no marker was changed, the same stock simulation is used)
```
ros2 run mission aruco_tracker.py
```
Run the mission
```
ros2 run mission mission.py
```


## DDS Implementation 

The DDS middleware is responsible for message transport between nodes.

### Why I switched from FastRTPS to Cyclone DDS

**FastRTPS (`rmw_fastrtps_cpp`)** caused errors like:

```
Change payload size ... cannot be resized
```

This is due to **fixed history depth** and **static payload sizes**. When PX4's messages exceeded limits (like `VehicleOdometry`), messages failed silently or crashed the node.

### Cyclone DDS (`rmw_cyclonedds_cpp`)

- Supports **dynamic memory allocation** for messages  
- Compatible with PX4’s default message definitions  
- Easy to switch:

``` bash
sudo apt install ros-$(ros2 pkg prefix rclcpp | xargs basename)-rmw-cyclonedds-cpp
```

```bash
echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> ~/.bashrc
source ~/.bashrc
```

---

## Mission Overview

### Stage 1: Cylinder Estimation (Autonomous Circle Flight)

1. Auto-arm and OFFBOARD mode
2. Take off vertically to (0, 0, -5)
3. Fly to (15, 0, -5)
4. Begin circular trajectory of radius 15 m, counter-clockwise
5. At each step, analyze `/drone/front_rgb` + `/drone/front_depth` for cylinders
6. After **one full revolution**, hover for 5 seconds

### Stage 2: Marker Landing (IBVS)

1. Fly to (0, 0, -13)
2. Fly to (0, 5, -13), hover for 5 s, record marker sample
3. Return to center
4. Fly to (0, -5, -13), hover for 5 s, record second marker
5. Compare both markers by depth → choose closest
6. Laterally approach selected marker
7. Perform **IBVS landing** using marker pixel offset and drone altitude

---

## Mission Algorithm (Pseudocode)

```text
# Cylinder Estimation
ARM → OFFBOARD
TAKEOFF to (0, 0, -5)
MOVE to (15, 0, -5)
START CIRCLE:
    θ = 0
    while θ < 2π:
        x = R*cos(θ)
        y = R*sin(θ)
        z = -5
        yaw = atan2(-y, -x)
        publish setpoint (x, y, z, yaw)
        detect cylinders
        θ += 0.02
HOVER 5 seconds

# Marker Landing
GOTO (0, 0, -13)
GOTO (0, +5, -13) → DETECT_FIRST
RETURN → (0, 0, -13)
GOTO (0, -5, -13) → DETECT_SECOND
RETURN → (0, 0, -13)
COMPARE marker₁.z vs marker₂.z
APPROACH closer marker
IBVS LANDING:
    repeat:
        compute error (x_m, y_m, z)
        apply control law
        update setpoint
    until ‖error‖ < 0.1
send LAND
```

---

## Mathematical Details

### 1. Cylinder Size Estimation

To compute real-world width and height from bounding box:

```math
w_m = \frac{w_{px} \cdot Z}{f_x}, \quad
h_m = \frac{h_{px} \cdot Z}{f_y}
```
where
Bounding box and intrinsics:
```math
w_{px}, h_{px} \quad \text{(bounding box pixel dimensions)};

Z \quad \text{(median depth)};

f_x, f_y \quad \text{(camera intrinsics)}
```
---

### 2. Circle Flight Path

The drone follows:

```math
x = R \cos\theta, \quad y = R \sin\theta, \quad z = -5
```

One full revolution is complete when:

```math
|\theta - \theta_0| \geq 2\pi
```

---

### 3. IBVS (Image-Based Visual Servoing) Control

#### Error Definition

```math
e_x = x_m \quad,
e_y = y_m \quad,
e_z = Z - Z_{\text{des}}
```
where
```math
(x_m, y_m) \quad \text{marker error in image space},
Z \quad \text{current drone altitude},
Z_{\text{des}} = 0 \quad \text{desired ground level}
```

#### Control Gains

```math
K_{\text{LAT}} = 0.1; \quad K_{\text{ALT}} = 0.05; \quad \Delta t = 0.1\; \text{s}
```

#### Velocity Commands

```math
v_x = -K_{\text{LAT}} \cdot e_x,
v_y = -K_{\text{LAT}} \cdot e_y,
v_z = -K_{\text{ALT}} \cdot e_z
```

#### Setpoint Update

```math
X_{\text{next}} = X + v_x \cdot \Delta t,
Y_{\text{next}} = Y + v_y \cdot \Delta t,
Z_{\text{next}} = Z + v_z \cdot \Delta t
```

#### Trigger Landing

```math
\sqrt{e_x^2 + e_y^2 + e_z^2} < 0.1 \Rightarrow \text{LAND}
```

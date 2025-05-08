# The Cart–Pole Problem

## State-Space Representation

The cart–pole system is modeled as a **linear time-invariant (LTI) system** with the state vector:

$$
x = \begin{bmatrix} 
x_c \\ 
\dot{x}_c \\ 
\theta \\ 
\dot{\theta} 
\end{bmatrix}
$$

Where:
- $x_c$ is the cart’s position.
- $\dot{x}_c$ is the cart’s velocity.
- $\theta$ is the pole’s angle from the vertical.
- $\dot{\theta}$ is the pole’s angular velocity.



The system is controlled by applying a force \( u \) to the cart, which affects the system dynamics.

---

## The Role of Q and R

### **Q Matrix: State Penalty**

The **Q matrix** determines how much deviations in each state is penalised. It is a **diagonal matrix**, where each element represents the weight assigned to a particular state variable. A larger value means that deviations in that state are more heavily penalised.

Using **Bryson’s Rule**,

$$
Q(i, i) = \frac{1}{(\text{max allowed state}_i)^2}
$$

This ensures that each state is normalized by its maximum permissible deviation.

For our system:

**The Q matrix is:**

![image](https://github.com/user-attachments/assets/9339abc0-dc83-4747-8654-fd7d5c0091a9)



This means:

- Deviations in **pole angle** are penalized the most (**weight of 25**).
- Cart position errors are also penalized significantly (**weight of 11.11**).
- Cart velocity and pole angular velocity are penalized less.

---

### **R Matrix: Control Effort Penalty**

The **R matrix** penalizes large control inputs (**force applied to the cart**). It is a **scalar value** in our case since only one control input is present:

$$
R = \frac{1}{\text{max force}^2}
$$

where:

- **Max force** = 15 N.

Thus:

$$
R = \frac{1}{15^2} = 0.0044
$$

A **small R value** means that applying force is not heavily penalized, allowing the controller to use strong corrections when needed.

---
## Implementation
${\color{red}A \space different\space approach\space than\space the\space last\space assignment\space where\space the\space explanations\space of\space the\space behaviour\space were\space more\space verbose\space than\space analytical\space.\space It\space will\space be\space toned\space down\space this\space time\space completely\space with\space easy\space to\space understand\space graphs\space doing\space most\space of\space the\space talking\space.}$

A barebones controller with live rqt tuning parameter to manually tune each value can be launched

```
ros2 launch cart_pole_optimal_control cart_pole_rviz.launch.py
```

Note that live update is one value behind, could not figure out why and didn't want to spend much time debugging.

The motivation behind the bryson values is the unwillingness to manually tune and log the values like the previous assignment as this takes much longer for one run and a higher range the values can be. 

So directly implementing the bryson values

```
ros2 launch cart_pole_optimal_control cart_pole_bryson.launch.py
```

[proper.webm](https://github.com/user-attachments/assets/f4594d2d-fbdc-4817-95f6-849fe916ec07)


These graphs are obtained
![control_input](https://github.com/user-attachments/assets/10e6e337-33cb-4cbb-9eca-a5a416c5ee9f)
![pole_angle_vs_cart_position](https://github.com/user-attachments/assets/006a6cac-903b-48a1-8bd1-ad8e007c7f48)
![state_evolution](https://github.com/user-attachments/assets/1d7e546a-8806-4986-9aab-8a0ef3c9970f)




## Multipliers

To understsand how different **Q** and **R** values a multiplier is added
```
ros2 launch cart_pole_optimal_control cart_pole_bryson_multipliers.launch.py
```

### Q x 1.5 and R x 0.8

[q15r08.webm](https://github.com/user-attachments/assets/c01304d4-c1b1-4ef7-98fd-5d8297cae321)


![q_values_vs_time](https://github.com/user-attachments/assets/cf7c95b7-d729-4a04-a8bd-a152c4394540)
![r_value_vs_time](https://github.com/user-attachments/assets/07b14ce3-5050-4341-87a0-772f8c51b3ff)
![control_input](https://github.com/user-attachments/assets/45a8e4ee-44cd-4605-9402-2c9c30a94d1a)
![pole_angle_vs_cart_position](https://github.com/user-attachments/assets/d9b9aa1e-154d-493e-8734-aaa28d65bb20)
![state_evolution](https://github.com/user-attachments/assets/14b34e63-7ca2-4bd7-8c0a-8c5d79f96688)


### Q x 0.8 and R x 1.5

[q08r15.webm](https://github.com/user-attachments/assets/7205f869-9dd2-4887-b6e9-a447ba6839a0)


![q_values_vs_time](https://github.com/user-attachments/assets/7618b14d-4ee1-4107-8fe8-ddd9bb64b19c)
![r_value_vs_time](https://github.com/user-attachments/assets/b1bf8a77-4069-47bd-a04e-a9cda0ed08db)
![control_input](https://github.com/user-attachments/assets/7a455994-9732-4174-a72f-c72920cd317f)
![pole_angle_vs_cart_position](https://github.com/user-attachments/assets/fe0f1a4e-520c-4e62-8688-0d400791c301)
![state_evolution](https://github.com/user-attachments/assets/7bad4b06-17f8-45cb-9307-f89dfb61060b)

## [Reinforcement Learning](https://github.com/blazair/spacerobotics/tree/main/assignments/assignment_2/RL)

A very peculiar error while I was trying to get the gym environment to work on ROS2. I had a venv with everything installed, but any node with gymnasium returned a **no module found** error.
But the same venv had no problem running a gymnasium environemnt outside ROS. Could not figure it out so I tried to learn RL separately. It is still an ongoing process, made quite easy because of this [source](https://pytorch.org/tutorials/intermediate/reinforcement_q_learning.html).

The error test was the [ralpoley](https://github.com/blazair/spacerobotics/tree/main/assignments/assignment_2/cartip_ws/src/ralpoley) package. Maybe I named the file the same as the import name, but the test was also done with a different name. I will revisit this when I have some free time. Any solutions please inform me.



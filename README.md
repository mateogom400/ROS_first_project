# Ackermann Steering and Bicycle Model for Vehicle Kinematics

This repository explores vehicle kinematic models commonly used in mobile robotics for localization and motion estimation. It includes implementations and explanations of:

- **Ackermann steering geometry** — accurately represents the physical behavior of four-wheeled vehicles.

- **Bicycle model approximation** — a simplified, widely-used alternative for odometry in robotics and autonomous systems.

The models are applied in the context of localization using proprioceptive sensors (such as wheel encoders and steering angle sensors) and were developed as part of a project at Politecnico di Milano.

---

# Overview
In autonomous mobile robotics—especially with car-like platforms—accurate pose estimation (position and orientation) is essential. When GPS or external localization systems are unavailable or unreliable, internal odometry becomes crucial.

Two standard kinematic approaches are:

- **Ackermann steering model**, capturing the true mechanics of front-wheel-steered vehicles.

- **Bicycle model approximation**, which simplifies the geometry to enable real-time estimation.

---

# Ackermann Steering Model

The Ackermann model describes the kinematics of typical four-wheeled vehicles where the front wheels steer independently. When turning, each front wheel follows a distinct trajectory, and the turning behavior depends on the vehicle's specific dimensions and steering configuration.

This model offers high fidelity, but its complexity makes it challenging for real-time applications.

---

# Bicycle Model Approximation

To reduce complexity, the bicycle model approximates the vehicle as having one steerable front wheel and one fixed rear wheel. This simplification retains the essential motion characteristics while being easier to implement in real time.

It is widely used in:
-Odometry and dead reckoning
-Path planning
-Real-time control and localization

Despite its simplicity, the model achieves effective results in many robotic and automotive scenarios—particularly when combined with sensor fusion techniques, such as the Extended Kalman 
Filter.

---

# Applications
This repository includes implementations for:
- Odometry estimation from wheel encoder and steering angle data
- Localization using the bicycle model
- ROS 1-compatible nodes for pose publishing
- Sensor fusion utilities, including GPS integration
- Tools to test and evaluate the accuracy of estimation systems
---

# Tools & Technologies
- ROS 1 (tested with Noetic)
- C++ / Python
- `nav_msgs/Odometry`, `tf`, `geometry_msgs`
- Extended Kalman Filter for improved estimation accuracy

---

# Academic Context
The models and algorithms are based on material from the Robotics course by Prof. Matteo Matteucci at Politecnico di Milano. This work was implemented during a student project focusing on vehicle localization along the Monza F1 circuit.

---

## Contact
**Mateo Gomez**
📧 [mateo.gomez@mail.polimi.it](mailto:mateo.gomez@mail.polimi.it)  
🔗 [linkedin.com/in/mateo-gomez-abril](https://www.linkedin.com/in/mateo-g%C3%B3mez-068694197/)

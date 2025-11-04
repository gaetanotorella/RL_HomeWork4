# Autonomous Navigation of a Differential Drive Mobile Robot

This project focuses on controlling a differential-drive mobile robot inside a simulated environment using ROS, Gazebo, and the standard mobile navigation stack. The objective is to construct the simulated world, place navigation references, configure the navigation system, and enable autonomous traversal between selected waypoints. Additionally, a vision-based extension is implemented using ArUco markers for marker-relative localization and navigation.

---

## Environment Setup

The simulation uses:
- **Gazebo** for the 3D environment and physics
- **ROS Navigation Stack** for global and local path planning
- **TF Framework** for referencing poses between coordinate frames

The robot is spawned in a predefined pose within the *rl_racefield* environment. An obstacle is repositioned, and a custom ArUco marker model is added so that the robot’s onboard camera can observe it when in proximity.

---

## Navigation Goals and Frame Broadcasting

A set of reference goals is introduced as **static TF frames** in the environment. These act as navigation targets. Their poses are retrieved at runtime through TF listening and are used as waypoints for autonomous travel.

The navigation task proceeds by:
1. Reading goal poses from static frames.
2. Sending goals sequentially to the navigation stack.
3. Using action feedback to determine whether the robot has reached each goal.
4. Continuing through the ordered navigation sequence.

The executed trajectory is recorded and can be visualized using rosbag playback and plotting tools.

---

## Autonomous Navigation

The **move_base** node computes velocity commands while accounting for:
- Global planning (map-level path search)
- Local planning (collision-aware reactive motion)
- Costmap layers (obstacle inflation and free-space estimation)

The navigation workflow follows a waypoint progression, demonstrating successful obstacle-aware motion and arrival detection.

---

## Adaptive Navigation Parameter Tuning

The navigation performance is influenced by:
- Dynamic obstacle distance thresholds
- Robot footprint and safety margins
- Velocity and acceleration bounds
- Local trajectory optimization constraints

Multiple parameter configurations are tested to observe effects such as:
- Passing through narrow corridors when clearance thresholds are reduced
- More conservative path selection when clearance thresholds increase
- Faster or slower motion depending on velocity and acceleration tuning
- Failure to reach goals if the robot footprint is expanded excessively

These adjustments illustrate how planner behavior changes in response to environment and safety trade-offs.

---

## Vision-Based Navigation Extension

To enable marker-relative navigation:
- The robot's camera is activated and calibrated in the URDF description.
- An ArUco marker is detected using the camera stream.
- The marker pose is expressed in the **map** frame using TF transforms.
- A new navigation target is defined relative to the marker’s position.
- The robot navigates autonomously to this relative target location.

This demonstrates a transition from *absolute goal navigation* to *vision-informed adaptive navigation*.

---

## Dependencies

- ROS Navigation Stack
- TF Transform Library
- Gazebo Simulation Environment
- ArUco ROS for marker tracking
- Standard mobile robot URDF and sensor configuration

---

## Output and Evaluation

- The robot correctly navigates between multiple predefined map goals.
- Parameter tuning modifies path planning behavior and clearance margins.
- Marker-relative navigation enables dynamic goal computation based on vision.
- Recorded bag files and trajectory plots confirm navigation performance across scenarios.

---
## Gallery
<img width="800" height="600" alt="Immagine3" src="https://github.com/user-attachments/assets/d6a90f82-70d9-4226-bdbb-36e9ca1924eb" />

<img width="800" height="600" alt="Immagine2" src="https://github.com/user-attachments/assets/204ccb2f-d9cf-4045-93c1-49287f95c567" />

<img width="500" height="600" alt="Immagine1" src="https://github.com/user-attachments/assets/6c1e099a-e968-4fa2-a8fc-04d06e6e8f37" />

## Demo Video

https://github.com/user-attachments/assets/349bf966-fe2c-4900-8a85-49931be1e9ec


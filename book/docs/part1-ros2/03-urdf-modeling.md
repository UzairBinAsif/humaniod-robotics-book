---
sidebar_position: 3
title: Humanoid URDF Modeling
---

# Humanoid URDF Modeling

Before we can simulate a robot or run any code on it, we need a standardized way to describe the robot's physical structure. In ROS, this is done using the **Unified Robot Description Format (URDF)**.

A URDF file is an XML file that describes the physical components of a robot, including its links, joints, sensors, and visual appearance.

## Core Components of URDF

### `<link>`

A **link** is a rigid body part of the robot. It has physical properties like mass and inertia, as well as visual properties that define how it looks in simulators like RViz. Each link has its own coordinate frame.

```xml
<link name="torso">
  <visual>
    <geometry>
      <box size="0.2 0.4 0.6" />
    </geometry>
  </visual>
  <collision>
    <geometry>
      <box size="0.2 0.4 0.6" />
    </geometry>
  </collision>
  <inertial>
    <mass value="10" />
    <inertia ixx="1.0" ixy="0" ixz="0" iyy="1.0" iyz="0" izz="1.0" />
  </inertial>
</link>
```

### `<joint>`

A **joint** connects two links together and defines how they can move relative to each other. Each joint has a parent link and a child link.

Key attributes of a joint include:

-   `name`: The name of the joint (e.g., "right_shoulder_pitch").
-   `type`: The type of motion allowed. Common types are `revolute` (for rotating joints), `continuous` (for wheels), `prismatic` (for sliding joints), and `fixed` (for rigid connections).
-   `parent` and `child`: The two links that the joint connects.
-   `origin`: The pose (position and orientation) of the child link's coordinate frame relative to the parent link's frame.
-   `axis`: The axis of rotation or translation for `revolute` or `prismatic` joints.
-   `limit`: Defines the motion range (for `revolute` and `prismatic` joints), including lower and upper bounds, effort, and velocity.

```xml
<joint name="torso_to_right_arm" type="revolute">
  <parent link="torso" />
  <child link="right_upper_arm" />
  <origin xyz="0 -0.3 0.2" rpy="0 0 0" />
  <axis xyz="0 1 0" />
  <limit lower="-1.57" upper="1.57" effort="100" velocity="1.0" />
</joint>
```

By connecting a series of links and joints, we can create a kinematic tree that represents the entire robot, from its base link all the way to its end-effectors. This URDF model is fundamental for simulation, motion planning, and visualization in ROS.

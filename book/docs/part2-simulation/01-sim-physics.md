---
sidebar_position: 1
title: Simulation Physics
---

# Simulation Physics in Gazebo & Isaac Sim

A robot's digital twin is only as good as the physics simulation it runs in. For a simulation to be useful, it must accurately model the forces that a real robot would experience, such as gravity, friction, and contact forces.

Both Gazebo and NVIDIA's Isaac Sim are powerful simulators, but they are built on different physics engines.

## Gazebo: Built on Open Dynamics Engine (ODE)

Gazebo, a long-standing tool in the ROS community, uses the **Open Dynamics Engine (ODE)**.

-   **What it is**: ODE is an open-source physics engine that is well-suited for simulating articulated rigid body dynamics—exactly what's needed for robotics.
-   **Strengths**: It is computationally efficient and has been battle-tested by the robotics community for over a decade. It's a reliable choice for many standard robotics applications.
-   **Limitations**: While fast, ODE's accuracy for complex contact and friction models can sometimes be limited compared to more modern engines.

## Isaac Sim: The Power of NVIDIA PhysX

NVIDIA Isaac Sim leverages **PhysX**, the same real-time physics engine that powers many of the world's most popular video games.

-   **What it is**: PhysX is a highly advanced, GPU-accelerated physics engine designed for high-fidelity, large-scale simulations.
-   **Strengths**:
    -   **Accuracy**: PhysX provides extremely accurate and stable simulations, especially for complex scenarios with many simultaneous contacts.
    -   **Performance**: By running on the GPU, PhysX can simulate incredibly complex worlds and robots with millions of particles, fluids, and deformable bodies in real time. This is something that CPU-based engines like ODE cannot match.
    -   **Advanced Features**: It includes advanced features like soft-body dynamics and high-precision friction models, which are critical for realistic grasping and manipulation tasks.

For this book, we will focus on **Isaac Sim** because its high-fidelity physics and tight integration with the NVIDIA AI stack make it the ideal choice for developing the next generation of physically-intelligent robots.

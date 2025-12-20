---
sidebar_position: 2
title: Isaac Sim Setup
---

# Isaac Sim Setup

NVIDIA Isaac Sim is a powerful robotics simulation toolkit built on the NVIDIA Omniverse™ platform. Setting it up involves a few key steps.

## 1. System Requirements

Before you begin, ensure your system meets the minimum requirements:
-   **OS**: Ubuntu 22.04
-   **NVIDIA Driver**: Version 525.60.11 or later.
-   **GPU**: An NVIDIA RTX™ desktop GPU (e.g., RTX 3070 or higher is recommended for good performance).
-   **RAM**: At least 32 GB.
-   **Storage**: At least 100 GB of free space.

## 2. Install NVIDIA Omniverse Launcher

Isaac Sim is installed and managed through the Omniverse Launcher.

1.  Visit the [NVIDIA Omniverse website](https://www.nvidia.com/en-us/omniverse/) and download the Omniverse Launcher for Linux.
2.  Make the downloaded `.AppImage` file executable:
    ```bash
    chmod +x omniverse-launcher-linux.AppImage
    ```
3.  Run the launcher:
    ```bash
    ./omniverse-launcher-linux.AppImage
    ```
4.  Follow the on-screen instructions to log in with your NVIDIA developer account and complete the setup.

## 3. Install Isaac Sim

Once the Omniverse Launcher is running:

1.  Navigate to the **Exchange** tab.
2.  Search for **Isaac Sim**.
3.  Click on the Isaac Sim application and select the latest version (e.g., 2023.1.1).
4.  Click **Install** to begin the download and installation process. This may take some time as the simulator is quite large.

## 4. Connecting ROS 2 to Isaac Sim

Isaac Sim has built-in support for ROS 2, allowing for seamless communication between your simulated robot and your ROS 2 nodes.

The connection is managed through the **ROS 2 Bridge**, which can be enabled and configured directly within the Isaac Sim interface. When active, the bridge automatically translates between ROS 2 messages and the simulation environment.

For example, a `sensor_msgs/msg/Image` message published from a simulated camera in Isaac Sim will appear on the ROS 2 network, ready to be consumed by any ROS 2 node. Similarly, a `geometry_msgs/msg/Twist` message published from your ROS 2 navigation code can be used to control the simulated robot's velocity.

In our labs, we will provide pre-configured environments where the ROS 2 bridge is already set up to connect to the appropriate topics for sensors and actuators.

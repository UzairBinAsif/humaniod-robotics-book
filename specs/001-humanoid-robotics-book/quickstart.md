# Quickstart: Humanoid Robotics Book

This guide provides instructions on how to build the book from source and run the code examples.

## Prerequisites

-   **OS**: Ubuntu 22.04
-   **ROS 2**: Humble Hawksbill
-   **NVIDIA Driver**: Version 525.60.11 or later
-   **Docker**: Latest version
-   **Node.js**: v18.x or later (for building the book)
-   **Python**: 3.10

## 1. Building the Book Website

The book is built using Docusaurus.

```bash
# Navigate to the book directory
cd book/

# Install dependencies
npm install

# Start the local development server
npm start
```

The book will be available at `http://localhost:3000`.

## 2. Running the Code Examples

The code for the labs is organized into separate ROS 2 workspaces. Each lab should be built and run independently.

### Example: Running Lab 1

```bash
# Navigate to the lab's workspace
cd code/lab1_ws/

# Build the ROS 2 workspace
colcon build --symlink-install

# Source the setup file
source install/setup.bash

# Launch the lab's ROS 2 application
ros2 launch my_package my_launch_file.launch.py
```

### Running Isaac Sim Labs (Lab 2 and later)

Labs involving Isaac Sim require the NVIDIA Omniverse Launcher and a local installation of Isaac Sim.

1.  **Install Isaac Sim**: Follow the official NVIDIA documentation to install Isaac Sim 2023.1 or later.
2.  **Set Up Environment**: Ensure your environment is configured to find the Isaac Sim Python executable.
3.  **Run the Lab**: The labs will include scripts to be executed with the Isaac Sim Python environment.

    ```bash
    # Example command
    ./_build/linux-x86_64/release/python.sh my_isaac_sim_script.py
    ```

Detailed instructions will be provided within each lab's README file in the `code/` directory.

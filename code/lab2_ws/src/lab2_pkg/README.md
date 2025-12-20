# Lab 2: Simulating a Robot in Isaac Sim

This lab is different from a standard ROS 2 package. The core logic will be implemented in Python scripts that are run using the Isaac Sim simulator's Python environment.

## Objective

To load a URDF model of a robot into Isaac Sim and control its joints using a standalone Python script.

## Instructions

1.  Ensure you have installed Isaac Sim via the Omniverse Launcher.
2.  Open Isaac Sim.
3.  Load the provided scene file (or follow the book's instructions to create one).
4.  Open the Script Editor within Isaac Sim (`Window` -> `Script Editor`).
5.  Paste the Python code from the book chapter into the editor.
6.  Run the script.

You will see the robot load into the scene and its joints will begin to move according to the logic in the script, which leverages the Isaac Sim APIs to apply joint commands.

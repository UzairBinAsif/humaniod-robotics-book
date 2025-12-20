---
sidebar_position: 1
title: LLM-to-Action Mapping
---

# LLM-to-Action Mapping

The true power of Physical AI emerges when a robot can understand and act on high-level, human-like commands. Simply telling a robot, "pick up the red block and put it on the blue one," is an incredibly complex task. It requires perception (identifying the blocks), planning (how to move the arm), and execution (controlling the motors).

This is where Vision-Language Models (VLMs) come in.

## From Language to Logic

A VLM, like the LLaVA model we selected in our research, can take both an image and a text prompt as input. It uses this combined context to reason about the scene and the instruction.

The core challenge is to map the VLM's natural language output into a concrete, executable action that our ROS 2 system can understand. This process is often called **grounding**.

## A Simple Voice-to-Action Pipeline

Let's consider a simple pipeline to translate a voice command into a robot action:

1.  **Speech-to-Text (Whisper)**: A user says, "Go to the red cube." OpenAI's Whisper transcribes the audio into the text string: `"Go to the red cube."`.

2.  **VLM Reasoning**: This text string, along with an image from the robot's camera, is fed into the VLM. The VLM's task is to identify the key components of the command:
    -   **Intent**: `go to`
    -   **Target Object**: `red cube`

3.  **Action Grounding**: Now, we need to translate the VLM's output into something our robot's navigation stack can use. This is where a custom "grounding" node comes in. This node's logic would be:
    -   It subscribes to the output of the VLM.
    -   It runs an object detection algorithm (e.g., from Isaac ROS) to find all objects in the current camera view.
    -   It finds the object that matches the description "red cube" and gets its `(x, y)` coordinates from the robot's map.
    -   If the intent is `go to`, it publishes this coordinate as a goal pose to the `/goal_pose` topic that Nav2 is listening to.

4.  **Execution (Nav2)**: Nav2 receives the goal pose and takes over, planning a path and moving the robot to the red cube.

This pipeline demonstrates how we can bridge the gap between abstract human language and the concrete world of robotic motion, allowing for a much more natural and intuitive way to interact with our machines.

### NoMaD with ROS2 Integration for Deployment on PX4-Autopilot in Gazebo Sim

[NoMaD: Goal Masked Diffusion Policies for Navigation and Exploration](https://arxiv.org/pdf/2310.07896) is a unified architecture that combines task-oriented navigation and task-agnostic exploration into a single diffusion policy. This enables robots to explore new environments effectively while navigating to user-specified goals when required.

## NoMaD Demo

The NoMaD has been successfully deployed for PX4-Autopilot. It can identify road paths and adjust the vehicle's yaw angle to align with the detected road direction. However, in certain scenarios, NoMaD occasionally selects suboptimal paths (e.g., the yellow path shown in the [demo video](https://drive.google.com/file/d/1Ra0ymba2Z3xgwwJOoG6dh5PbuQMNfDAU/view?usp=sharing)) that result in collisions.

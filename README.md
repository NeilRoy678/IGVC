# Intelligent Ground Vehicle Competition (IGVC) Project

## Overview
This project was developed for the **Intelligent Ground Vehicle Competition (IGVC)**, where we built an autonomous ground vehicle capable of navigating dynamic environments with automated lane detection, pothole avoidance, and lane adjustments. The system was developed using **Robot Operating System (ROS)**, computer vision, machine learning, and control algorithms.

## Key Features

- **Lane Detection & Adjustment**: 
  - Developed vision algorithms, including **Inverse Perspective Mapping**, to detect road lanes.
  - Integrated a **PID controller** for automated lane adjustment, enabling the vehicle to follow the path without manual intervention.

- **Pothole Detection**: 
  - Implemented a machine learning model to detect potholes using camera feeds.
  - The system was trained and simulated in ROS, allowing the vehicle to detect and avoid potholes autonomously.

- **Cost Optimization**: 
  - Replaced expensive LiDAR sensors with **vision-based algorithms** like **Gaussian Potential Field** for obstacle detection, significantly reducing project costs while maintaining accuracy.

## Technologies Used

- **Robot Operating System (ROS)**: Framework for integrating subsystems and algorithms.
- **Computer Vision**: Techniques like inverse perspective mapping for lane detection and Gaussian Potential Field for obstacle avoidance.
- **Machine Learning**: Trained model for pothole detection and avoidance.
- **PID Control**: Used for automated lane following and path adjustment.

- https://user-images.githubusercontent.com/74729526/197328961-532bbfe1-5323-418d-87ef-c90a70f1567d.mp4

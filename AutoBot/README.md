# METHODS FOR NAVIGATION
## 1. Using Pointcloud Data
  - Detect Lanes and obstacles using depth camera and converting it into pointcloud
  - Taking and filtering lanes pointclouds and stacking them to create virtual walls
  - Feeding the processed pointcloud data into a planner
    - Problem with using ros planners is that they need a global map so check if there is a way around
    - We can create our own planner which avoids obstacles and prevent backtracking since it is already in a closed space due to virtual walls

## 2. Using Ros Planner 
  - Detect obstacles and Lanes using camera and lidar and convert it into Bird's eye view(using IPM) and generate a costmap
  - For the global planner we create a temporary map of the surrounding area(for eg in a 5m radius) and we update it everytime we are about to exit the area
  - we will use teb planner for this

## 3. Using a Potential Field Approach
  - In this method we assign artficial potential fields to objects, the idea is that waypoints will have an attractive force and te obstacles will have a repulsive force. We move in the direction of the attractive force. [Link to the paper](https://www.hindawi.com/journals/jat/2018/5041401/)
  - Again, for this we would also need a managerial node to prevent the robot from exiting the lanes and avoid backtracking as the potential field would only help us in obstacle avoidance.

## 4. The Very Basic
  - Make a simple lane follower and a obstacle avoidance node that just executes without any planner

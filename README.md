
# Project Clearpath
ENPM700-Final Project

# C++ Boilerplate v2 Badges
![CICD Workflow status](https://github.com/sriramprasadkothapalli/project_clearpath/actions/workflows/run-test-and-upload-codecov.yml/badge.svg) [![codecov](https://codecov.io/gh/sriramprasadkothapalli/project_clearpath/graph/badge.svg?token=0NZGAQ9FZ2)](https://codecov.io/gh/sriramprasadkothapalli/project_clearpath) [![License](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)

| Role / Part   | Phase 1                   | Phase 2                   | Phase Testing             |
|---------------|---------------------------|---------------------------|---------------------------|
|Driver         |Sriram Prasad Kothapalli   |Tathya Bhatt               |Bhavana B Rao              |
|Navigator      |Bhavana B Rao              |Sriram Prasad Kothapalli   |Tathya Bhatt               |
|Design Keeper  |Tathya Bhatt               |Bhavana B Rao              |Sriram Prasad Kothapalli   |



"Project Clearpath" is an autonomous robot designed for disaster site cleanup, capable of identifying, and localizing hazardous debris. It autonomously navigates sites, detecting debris types, and estimating 3D locations.
This repository contains deliverables for final project of Bhavana B Rao, Sriramprasad Kothapalli and Tathya Bhatt as a part of the course ENPM700: Software Development for Robotics at the University of Maryland.

## AIP

This project was developed using the Agile Development Process (AIP) along with pair programming (with a driver, navigator, and design keeper), with a focus on test-driven development (TDD). This [sheet](https://docs.google.com/spreadsheets/d/124zPjeAy8mCFpvf6AqBxNpdo9hfX9Y0hR05A0yxhxY0/edit?gid=0#gid=0) has the product backlog, iteration backlogs, and work log for each task done. The link to the sprint planning and review meeting notes devised to overview each iteration sprint is attached [here](https://docs.google.com/document/d/1AVcXsBQ32G4zNruy3SwoopeVLuxpyloBtEbVzfRSro4/edit?tab=t.0#heading=h.mw18pfbxwsdy).

## UML

The initial UML Diagram designed during proposal phase is listed under UML/initial. The UML diagram for the Phase-1 and Phase-2 implementation is under UML/revised.

## Directory Structure and Contents

```bash
project_clearpath/ 
├── app 
|   └── main.cpp # Entry point for the robotic system 
├── launch 
|   └── collector_robot.launch.py # Launch file to initialize the robot system 
├── libs 
|   └── debris 
|     ├── debris_detection.cpp # Implementation of debris detection functionality 
|     ├── debris_detection.hpp # Header file for debris detection 
|     ├── debris_removal.cpp # Implementation of debris removal functionality 
|     └── debris_removal.hpp # Header file for debris removal 
├── test/ │ 
|  ├── test_level1.cpp # Unit tests for individual components (Level 1) 
|  └── test_level2.cpp # Integration tests for full system functionality (Level 2)
```

### Debris Detection System

The debris_detection.cpp contains the `DebrisDetector` node implementation, which uses ROS2 and OpenCV to detect and navigate towards debris in a robot's field of view. The system integrates image processing, odometry handling, and navigation for autonomous operation.

#### Features
- Detect debris using color-based thresholds in the HSV color space.
- Navigate towards detected debris while avoiding obstacles.
- Publish debug images for visualization.
- Modular design with ROS2 publishers, subscribers, and services.

### Debris Removal System

This debris_removal.cpp contains the `DebrisRemover` node implementation, which uses ROS2 services to simulate the removal of detected debris. The system integrates with Gazebo simulation to "clean up" debris, representing the autonomous removal of obstacles from the environment.

#### Features
- Interface with Gazebo's /delete_entity service to remove debris from the simulation.
- Modular design to seamlessly integrate with the DebrisDetector node for end-to-end autonomous operation.
- Simulates real-world cleanup by removing specified objects from the robot's workspace.

### Level 1 Test

Focuses on unit testing individual components like debris detection and removal, ensuring isolated functionality using the Google Test framework.

### Level 2 Test

Performs integration tests to validate interactions between components, simulating the full robotic system using Google Test.

## Dependencies
Ensure the following dependencies are installed before building the package:

### General Dependencies
- **ROS2 (Humble/Other compatible distribution)**
- **OpenCV** (>=4.x)
- **cv_bridge**
- **image_transport**
- **rclcpp**

### Testing and Coverage Dependencies
- **lcov**
- **genhtml**

Install the required dependencies using your system package manager or ROS2 tools.


## Build Instructions

1. Clone this repository into your ROS2 workspace:
   ```bash
   cd ~/ros2_ws/src
   git clone <repository_url>
   ```

2. Navigate to the root of your workspace and build the package with coverage flags:
   ```bash
   cd ~/ros2_ws
   colcon build --cmake-args -DCMAKE_CXX_FLAGS="--coverage" -DCMAKE_C_FLAGS="--coverage" -DCMAKE_BUILD_TYPE=Debug
   ```

3. Source the workspace:
   ```bash
   source install/setup.bash
   ```

### Testing

Run the unit and integration tests for the package:
```bash
colcon test --event-handlers console_direct+
```

If all tests pass, proceed to generate the coverage report.


### Generate Coverage Report

1. Capture the coverage data:
   ```bash
   lcov --capture --directory build --output-file coverage.info
   ```

2. Filter out irrelevant files:
   ```bash
   lcov --extract coverage.info '*/libs/debris/*' '*/test/*' --output-file filtered_coverage.info
   ```

3. Generate an HTML report:
   ```bash
   genhtml coverage_filtered.info --output-directory coverage_report
   ```

4. Open the coverage report in your browser:
   ```bash
   firefox coverage_report/index.html
   ```

### Launch 

To launch the node, run the following command:
```bash
ros2 launch project_clearpath collector_robot.launch.py
```

A demo video for Phase 1 that tests only the debris detection system is as follows: 

[Watch the demo video](https://drive.google.com/file/d/1Fyi_XzjUzuUIOucaIdy7Z6o0Lr2dtfaS/view?usp=sharing)

The complete working video simulating a turtlebot in a disaster environment identifying and cleaning the obstacles is as given: 

[Video] (https://umd.box.com/s/bzaexvmz96letgl0csp6mganamysl0pr)

![Simulation Environment] (images/Gazebo.png)

## License

This project is licensed under the MIT License. See the `LICENSE` file for more details.















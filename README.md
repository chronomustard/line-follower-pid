# Line Follower PID

This repository contains the codebase for a line follower robot designed for the ENEE607404 class (2023). The project facilitates the control of a line follower robot built using ESP32, connected to encoders, PMDCs (Permanent Magnet DC Motors), and IR sensors. It leverages error data from IR sensors to regulate the speed of the left or right PMDCs.

## Project Structure

- **.vscode**: Configuration files for Visual Studio Code.
- **data**: Data files, configurations, and website for the Line Follower PID control project.
- **include**: Header files or additional includes.
- **lib**: Libraries or external dependencies used in the project.
- **src**: Source code directory containing code for controlling the line follower robot.
- **test**: Test cases, unit tests, or test-related files.
- **.gitignore**: Git configuration file specifying ignored files and directories.
- **platformio.ini**: PlatformIO configuration file defining project settings and environment.

## Overview

The Line Follower PID project aims to implement a PID (Proportional-Integral-Derivative) control system for a line-following robot. Equipped with IR sensors, the robot detects lines on the surface, utilizing this data to make directional decisions.

## Components Used

- **ESP32**: Core microcontroller for processing.
- **Encoders**: Measure wheel rotation and movement.
- **PMDCs (Permanent Magnet DC Motors)**: Actuators for robot movement.
- **IR Sensors**: Detect lines on the surface for navigation.

## Functionality

The project's core functionality revolves around processing error data obtained from IR sensors. This data is crucial for adjusting the speed of the left or right PMDCs. The adjustment allows the robot to effectively follow lines with precision.

## Usage

1. **Setup ESP32**: Ensure proper connections of the ESP32 to components (encoders, PMDCs, IR sensors) as per provided specifications.

2. **Upload Code**: Use the code provided in the [src/](src/) directory to upload onto the ESP32.

3. **Calibration**: Perform necessary calibration as documented for the project.

4. **Execution**: Power on the robot and initiate the line-following sequence. The robot should commence following lines based on the implemented PID control.

## Contribution

Contributions aimed at enhancing functionality, optimizing code, improving documentation, or addressing issues are welcome! Fork this repository, make changes, and submit a pull request.

## Note

This project serves as a guide for those interested in similar projects. Please note that complete schematics for this project might not be available as it hasn't been fully provided to the students.

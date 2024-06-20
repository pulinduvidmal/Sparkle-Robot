# Sparkle-Robot 🤖

This repository includes the complete source code, design files, and documentation for the Sparkle Robot, a versatile and feature-rich automation platform built for the EN2533 - Robot Design and Competition module.

<div align="center">
  <img src="/docs/assets/robot.png" alt="New Image" title="New Image" style="width: 500px; max-width: 100%;" />
</div>

## Features

* **Line Following & Obstacle Avoidance 🚧:** Enables the robot to navigate predefined paths while avoiding obstacles using phototransistors and ultrasonic sensors.
* **Ramp Navigation & Box Dragging 📦:**  A robust mechanical design with a geared motor system enables the robot to ascend ramps and drag boxes.
* **Box Picking & Color Detection 🎨:** A custom SolidWorks-designed robotic arm, powered by a servo motor, allows for precise box picking. A color sensor integrated into the arm enables the robot to differentiate between different colored boxes.
* **Maze Solving 🌀:** Employs a `left-hand rule algorithm`, guided by color sensing, to navigate mazes. The robot utilizes the box color to determine the correct path and destination within a maze environment.
* **Sound Detection 🎤:** Incorporates an audio sensor that allows the robot to detect and respond to specific sound cues, enabling dynamic interactions and programmed actions based on sound triggers.

## Repository Structure

```
├── design
├── docs
│   └── assets
|   └── requirements
├── src
└── scripts
```

**Detailed Description:**

* **design/Robot-Arm-STL:** Contains STL files for 3D printing the robotic arm components designed using SolidWorks.
* **docs**
    * **assets:** Contains supplementary assets for documentation, such as images, diagrams, and videos.
    * **requirements:**  Contains competition requirements documents for the EN2533 module. This folder should include:
        * **Task_v1.pdf:**  The initial version of the competition guidebook.
        * **Task_v1.1.pdf:**  An updated version of the competition guidebook with revisions and clarifications.
* **src:**  contains the Arduino source code for the Sparkle Robot.
    * **sparkle_robot_src.ino:**  The main Arduino program that controls all the robot functionalities.
* **scripts:** Contains scripts for each hardware part of robot for test their funcationality separately.

## Task - 2023

> [!TIP]
> The full task documentation can be found in the [requirements](/docs/requirements/) folder.
> - [Task_V1](/docs/requirements/EN2533-Robot_Designand_Competition_Task_v1.0.pdf)
> - [Task_V1.1](/docs/requirements/EN2533-Robot_Design_and_Competition_Task_v1.1.pdf)

<div align="center">
  <img src="/docs/assets/arena.png" alt="Arena" title="New Image" style="width: 500px; max-width: 100%;" />
</div>

## Robot overview

### Adding Functionality

* **Line Following:**
    * Implement a line-following algorithm using ir array.
    * Use the analogRead() function to read sensor values.
    * Implement a PID (Proportional-Integral-Derivative) controller to maintain a stable line-following trajectory.
* **Obstacle Avoidance:**
    * Utilize ultrasonic sensors to detect obstacles in front of the robot.
    * Implement a basic obstacle avoidance logic to stop or steer the robot away from detected objects.
* **Ramp Navigation:**
    * Ensure a robust mechanical design that can handle the weight and friction of ascending ramps.
    * Implement a motor control algorithm that can adjust motor speeds to smoothly navigate ramps.
* **Box Dragging:**
    * Design a mechanism that allows the robot to grip and drag boxes without dropping them.
    * Implement a motor control system that can accurately control the speed and direction of the dragging motion.
* **Box Picking & Color Detection:**
    * Design a robotic arm in SolidWorks with appropriate dimensions and articulation.
    * Utilize a servo motor to control the arm movement and gripping mechanism.
    * Integrate a color sensor into the arm's gripping mechanism to detect the color of the box.
* **Maze Solving:**
    * Implement the left-hand rule algorithm to navigate mazes.
    * Use the color sensor to identify the correct path based on the color of the box being carried.
    * Develop logic to determine the correct destination for placing the box.
* **Sound Detection:**
    * Utilize an audio sensor to detect specific sound cues.
    * Implement logic to trigger specific actions or movements based on detected sounds.


### Hardware Requirements

**Sensors:**

* **[Raykha S8 – TCRT5000 based 8 channel Reflective Sensor Array:](https://aptinex.com/product/raykha-s8/)**
    * **Function:** Line detection and color differentiation.
    * **Application:** Line following.
* **[HC-SR04 Ultrasonic Sensors](https://lastminuteengineers.com/arduino-sr04-ultrasonic-sensor-tutorial/):**
    * **Function:** Distance measurement.
    * **Application:** Obstacle detection, object picking, and guarding.
* **[TCS230 RGB Color Recognition Sensor](https://randomnerdtutorials.com/arduino-color-sensor-tcs230-tcs3200/):**
    * **Function:** Color detection and differentiation.
    * **Application:** Competition box object colors identification.
* **[MAX9814 Electret Microphone Amplifier Stable Module with Auto Gain Control](https://www.phippselectronics.com/using-the-max9814-microphone-amplifier-module-with-arduino/):**
    * **Function:** Sound detection and threshold adjustment using [arduinoFFT](https://github.com/kosme/arduinoFFT) library.
    * **Application:** Interactive functionality, event triggering.
* **[MPU6050 Accelerometer and Gyroscope](https://lastminuteengineers.com/mpu6050-accel-gyro-arduino-tutorial/):**
    * **Function:** 3-axis motion tracking.
    * **Application:** Motion tracking, orientation sensing.
      
**Actuators:**

* **DC Gear Motors (34:1 metal gear motors):**
    * **Voltage:** 12V
    * **Application:** Linear motion, driving wheels, and providing torque.
      
* **Servo Motors (MG996R metal gear high torque servo motors):**
    * **Voltage:** 6V
    * **Rotation:** 180 degrees
    * **Application:**  Robotic arm movement, gripping mechanisms, and slider operation.

**Power and Battery:**

* **Battery:** 3S 5200mAh 40C 11.1v LiPo battery.
* **Power Management:**
    * **Step-down buck converter (LM2596S):**  Reduces battery voltage to 9V for motor driver module operation.
    * **Independent buck converter:**  Converts voltage to 5V for sensor functionality.

**Indicators:**

* **LEDs:**  Indicate infrared sensor status and provide visual feedback on sensor operation.

**Other Components:**

* **Power Switch:**  Enables on/off control of the entire robot system.
* **Task-Specific Switches:**  Control activation of specific robot functionalities during competition tasks.

**Robot Arm and Mechanisms:**

* The robot arm is designed in SolidWorks and is powered by a servo motor.
* The arm is responsible for picking up and placing boxes, and it incorporates a color sensor for identifying box colors.
* Gripping mechanisms and slider operations are controlled by servo motors, enabling precise manipulation of objects.

**Software:**

* **[Arduino Mega 2560](https://docs.arduino.cc/hardware/mega-2560/):**  The main microcontroller that controls all robot functionalities.
* **Source Code:**  The code for the Sparkle Robot is located in the `src` folder. The main program file is `sparkle_robot_src.ino`.

> [!TIP]
> Individual hardware components and their functionalities can be independently verified using dedicated test scripts located within the `scripts` folder.

> [!WARNING]
> Modifying hardware component connections or device configurations will inevitably necessitate adjustments to the corresponding pin configurations within the source code or scripts.
## Team Members

- [**Akhila Prabodha**](https://github.com/AkhilaPrabodha)
- [**Pulindu Vidmal**](https://github.com/pulinduvidmal)
- [**Navini Jagoda**](https://github.com/Navini11)
- [**Eshan Surendra**](https://github.com/eshansurendra)
- [**Achira Hansindu**](https://github.com/achira1428gsd)
  
## Demo

https://github.com/pulinduvidmal/Sparkle-Robot/assets/107745680/13c5de56-0cc0-490a-96d4-449374089251

## Contributions

Contributions are welcome! 

- **Bug Fixes:** If you find any bugs or issues, feel free to create an issue or submit a pull request.
- **Feature Enhancements:** If you have ideas for new features or improvements, don't hesitate to share them.

Feel free to reach out with any questions or suggestions!

# License

This project is licensed under the [MIT License](LICENSE).

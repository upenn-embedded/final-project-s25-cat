[ ![Review Assignment Due Date](https://classroom.github.com/assets/deadline-readme-button-22041afd0340ce965d47ae6ef1cefeee28c7c493a6346c4f15d667ab976d596c.svg)](https://classroom.github.com/a/WXeqVgks)

# final-project-skeleton

* Team Number: 1
* Team Name: CAT
* Team Members: Chekayli Meyer, Andrea González, Taarana Jammula
* GitHub Repository URL: [upenn-embedded/final-project-s25-cat: ese3500s25-final-project-s25-final-project-skeleton created by GitHub Classroom](https://github.com/upenn-embedded/final-project-s25-cat)
* GitHub Pages Website URL: [for final submission]

## Final Project Proposal

### 1. Abstract

*In a few sentences, describe your final project.*

Our final project involves developing an autonomous trash collection vehicle that detects, collects, and stores small lightweight objects while navigating its environment. The robot moves forward until an object is detected within a few centimeters, at which point it stops, collects using a shovel-like part, and places it near a beacon. The system would use an IR sensor for object detection and an ultrasonic sensor for wall detection.

### 2. Motivation

*What is the problem that you are trying to solve? Why is this project interesting? What is the intended purpose?*

We want to address the growing need for small-scale automated cleanup solutions in both indoor and outdoor environments. We are particularly interested in applying embedded systems and robotics principles to create a functional and modular trash-collection vehicle. This project will allow us to integrate course topics such as sensors, actuator control, serial communication, and embedded C programming in a hands-on and practical setting. Beyond the course requirements, we aim to continue building the system with real-time object recognition and autonomous navigation to explore more advanced robotics applications.

### 3. System Block Diagram

*Show your high level design, as done in WS1 and WS2. What are the critical components in your system? How do they communicate (I2C?, interrupts, ADC, etc.)? What power regulation do you need?*

![1742761278310](image/README/1742761278310.png)

The critical components of our system are the battery, buck converter, ultrasonic sensor, IR sensor, Raspberry Pi, camera, the ATMega328Pb, the L298N H-Bridge Driver, and 2 motors. The camera will be connected to the raspberry pi and will send information to the ATMega328Pb via I2C (serial communication). Timer0 will use output compare to detect walls, Timer1 and Timer2 will be connected to separate motors once an object is detected (to push the object to the beacon). Interruopts

### 4. Design Sketches

*What will your project look like? Do you have any critical design features? Will you need any special manufacturing techniques to achieve your vision, like power tools, laser cutting, or 3D printing?*

![1742760587519](image/README/1742760587519.png)

### 5. Software Requirements Specification (SRS)

*Formulate key software requirements here. Think deeply on the design: What must your device do? How will you measure this during validation testing? Create 4 to 8 critical system requirements.*

**5.1 Definitions, Abbreviations**

**ATmega328PB:** Microchip ATmega328PB microcontroller which is an 8-bit AVR MCU

**IR:** Infrared which is the proximity or reflectance sensor

**US:** Ultrasonic Sensor which will be used for distance measurement like in Lab 3

**PWM:** Pulse Width Modulation for motor speed control

**ADC:** Analog-to-Digital Converter

**I2C:** Inter-Integrated Circuit for two-wire serial communication protocol

**GPIO:** General Purpose Input/Output

**5.2 Functionality**

| **Req ID** | **Software Requirement**                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    | **Verification Method**                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |
| ---------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **SRS-01** | Sensor Polling Frequency: The firmware shall poll the IR sensor and ultrasonic sensor at least 10 times per second (≥10 Hz each) to ensure timely detection of trash and obstacles.                                                                                                                                                                                                                                                                                                                                                                                              | Test: Use a logic analyzer or oscilloscope to verify sensor read operations (or an LED toggled in code on each read) occur at ≥10 Hz. Confirm that both IR and US sensor readings are updated at 100 ms intervals or faster under normal operation.                                                                                                                                                                                                                                                                                            |
| **SRS-02** | Trash Detection Threshold: The system shall detect a piece of trash when the IR sensor’s reading exceeds a calibrated threshold value. Upon threshold crossing, the microcontroller shall initiate the trash collection routine (e.g. stop forward motion and activate the scoop) within 100 ms.                                                                                                                                                                                                                                                                                 | Test:Place a standard test object (simulated trash) at the detection range limit (e.g. ~15 cm in front of the IR sensor) and verify that the firmware recognizes it (IR reading above threshold) and triggers the scoop mechanism within 0.1 s. Repeat across several distances and object sizes to confirm reliable threshold detection.                                                                                                                                                                                                       |
| **SRS-03** | Obstacle Avoidance Threshold: The firmware shall use the ultrasonic sensor to detect walls or obstacles within a set distance (e.g. 30 cm). If an obstacle is closer than the threshold, the system shall halt or turn away within 100 ms to avoid collision.                                                                                                                                                                                                                                                                                                                     | Test:Gradually bring a wall or large object toward the robot and confirm that when it crosses the ~30 cm range, the robot’s motors stop or turn immediately (within 0.1 s). Use a measuring tape to ensure the trigger distance is ~30 cm and a timer or high-speed camera to measure response time from detection to motor stop.                                                                                                                                                                                                              |
| **SRS-04** | The Raspberry Pi shall capture images using an attached camera at a rate of at least 1 frame per second while scanning for trash or the beacon. It shall process the image and send detection results to the ATmega328PB via I2C within 500 ms.                                                                                                                                                                                                                                                                                                                                   |                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 |
| **SRS-05** | PWM Motor Control Timing: The firmware shall drive the motors using a PWM signal of at least 100 Hz frequency (with a target around 500 Hz for smooth control).The duty cycle shall be updated as needed (on speed changes) with a control loop period not greater than 100 ms. This ensures smooth speed variation without perceptible stalling or jitter.                                                                                                                                                                                                                       | Test:Measure the PWM output on the motor driver input using an oscilloscope. Verify the frequency is ≥100 Hz (e.g. ~500 Hz achieved, within ±5% of target). Change speed commands and confirm the duty cycle adjusts within the next 100 ms cycle. No audible irregularities or excessive motor vibration should be observed at the set frequency (confirm by listening and motor behavior).                                                                                                                                                  |
| **SRS-06** | I2C Communication Protocol: The microcontroller shall communicate with the Raspberry Pi using the I2C bus operating at standard speed (100 kHz clock). Sensor status updates (e.g. trash detected, weight, obstacle distance) shall be transmitted to the Pi at least 2 times per second, and command/control data from the Pi (e.g. navigation commands) shall be processed within 100 ms of receipt. The I2C communication shall include error-checking (ACK/NACK monitoring); on a communication failure, the system shall retry the transmission at least once within 50 ms. | Test:Use a protocol analyzer or logic analyzer on the I2C lines (with level shifting in place) to verify the bus clock is ~100 kHz. Run the system and confirm via logs that sensor data messages are sent to the Pi ≥2 Hz. Introduce a forced NACK or bus error (e.g. by briefly disconnecting SDA) and ensure the firmware attempts a retry and recovers communication. All commands from the Pi (simulated via I2C commands) should be observed to take effect on the robot (e.g. a movement command causes motor PWM change) within 0.1 s. |
| **SRS-06** | User Interrupt & Safety Response: The system shall provide a user interrupt (e.g. an emergency stop button or remote kill switch input to a GPIO interrupt). When the user interrupt is activated, the firmware shall immediately override normal operation and stop all motor activity within 100 ms, entering a safe idle state. Normal operation can only resume after a manual reset or explicit resume command.                                                                                                                                                             | Test:While the robot is moving, activate the emergency stop (press the button or trigger the interrupt line). Measure the time for the motors to stop (e.g. using timestamps or an LED triggered at stop); it should be <100 ms. Verify that during the stop condition, no motor motion occurs and the robot remains stationary. Attempt to issue movement commands during the stop state to ensure they are ignored. Finally, reset the interrupt and confirm the system only resumes operation when allowed.                                  |

### 6. Hardware Requirements Specification (HRS)

*Formulate key hardware requirements here. Think deeply on the design: What must your device do? How will you measure this during validation testing? Create 4 to 8 critical system requirements.*

*These must be testable! See the Final Project Manual Appendix for details. Refer to the table below; replace these examples with your own.*

**6.1 Definitions, Abbreviations**

**H-Bridge** An electronic circuit (L298N module) that drives the motors in both directions

**Beacon:** Wireless beacon used as a drop-off location marker (as in emitting a signal for robot to home in)

**Battery:** On-board power source (rechargeable) for motors and electronics

**6.2 Functionality**

| **Req ID** | **Hardware Requirement**                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             | **Verification Method**                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 |
| ---------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **HRS-01** | Drive Motor Performance:The motor and drive system shall propel the robot at a forward speed of≥0.3 m/son level ground while carrying a full load of collected trash (at least 2 kg). Each motor (with L298N driver) must provide sufficient torque to start moving from standstill with the full load and climb a mild incline of 5°.                                                                                                                                                                                                                                   | Test: Load the robot with a dummy weight of 2 kg (to simulate trash). Mark a 1 m distance on flat floor and time the robot driving that distance; it should arrive in ≤3.3 s (0.3 m/s or faster). Verify the robot can also ascend a 5° ramp from standstill with the same load without stalling. Monitor motor current draw with an ammeter during these tests to ensure it stays within L298N and motor limits (no overheating or current limiting observed).                                                                                                                                                                                                            |
| **HRS-02** | Infrared (IR) Sensor Range & Accuracy: The IR sensor shall reliably detect a typical piece of trash (minimum size ~5 cm) at a distance of≥15 cm in front of the robot. Detection reliability at that range should be at least 90% (minimal missed detections under proper conditions). Beyond 15 cm, the sensor should not trigger on the test object, to avoid false positives outside the intended range.                                                                                                                                                              | Test: Place a standard test object (e.g. a 5 cm cube or bottle) at 15 cm from the IR sensor and observe the sensor output over 10 trials (approaching and leaving). It should correctly indicate “object present” in at least 9 out of 10 trials at 15 cm. Repeat at slightly greater distances (20 cm, 30 cm) to ensure the sensor does not indicate detection beyond ~15 cm. Also test against different materials/backgrounds to confirm consistent performance and that environmental IR (lighting) does not cause false detection.                                                                                                                                     |
| **HRS-03** | Ultrasonic Sensor Distance & Accuracy:The ultrasonic sensor shall measure distances to obstacles in the range of5 cm up to 300 cm. Within 100 cm, the distance measurement error should not exceed±1 cm , and for longer ranges (up to 300 cm) error should be within ±5 cm . (The sensor’s capability is 2–400 cm with ~0.3 cm precision in ideal conditions) This accuracy is required for reliable wall detection and navigation.                                                                                                                                   | Test: Place a flat obstacle (e.g. a wall or board) at known distances of 10 cm, 50 cm, 100 cm, 200 cm, and 300 cm from the ultrasonic sensor. For each distance, take multiple readings (e.g. 10 samples) and record the measured distances. Calculate the error compared to the true distance – it should be within ±1 cm for ≤100 cm ranges and ±5 cm for the farthest distances. Also verify the sensor consistently detects an object at 300 cm (does not miss it) and that it reports out-of-range when no object is within ~3 m.                                                                                                                                   |
|                  |                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            |                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               |
| **HRS-04** | Battery and Power System:The robot shall be powered by a battery system that provides sufficient voltage and capacity for both motors and electronics. The battery pack (and regulators) must supply12 V(nominal) for the motors and a regulated5 V±5%for the microcontroller and sensors. The capacity shall support at least1 hourof continuous operation (motors driving and sensors active) before recharge. The 5 V regulator should handle peak current draw (motor drivers + logic, ~2–3 A transient) without the voltage dropping out of the 4.75–5.25 V range. | Test: Use a multimeter to verify the battery voltage and regulator outputs. Under no-load, confirm ~12 V from the battery and 5 V at the logic rail. Then run the robot under a heavy load scenario (e.g. motors stalled or frequent start-stop to draw peak current) and measure the 5 V line stays within 4.75–5.25 V. Check that the system runs for at least 60 minutes on a full charge by driving it continuously in an obstacle course; record the run time until the battery depletes to cutoff. Also observe that the microcontroller and sensors operate normally throughout, indicating the regulator is supplying stable power (no resets or brown-outs).        |
| **HRS-05** | Logic Level Interface (I2C Bus Hardware): The I2C level shifter and bus wiring shall allow reliable communication between the 5 V ATmega328PB and 3.3 V Raspberry Pi. The hardware must not introduce signal distortion at the standard 100 kHz I2C clock. Pull-up resistors or the level shifter module shall maintain the I2C lines within proper voltage thresholds for both devices. Communication integrity shall be ≥99% (no more than 1% packet loss or corruption during normal operations of 10 minutes).                                                       | Test: Instrument the SDA and SCL lines with an oscilloscope while the microcontroller and Pi exchange data. Verify the logic high level on the bus is ~3.3 V (through the level shifter) and rise/fall times are within I2C specifications at 100 kHz. Run a continuous I2C communication test for 10 minutes (transferring hundreds of messages) and count any errors or checksum failures in the data; the error rate should be <1%. Also test the bus in both idle and motor-active scenarios (to ensure motor electrical noise doesn’t disturb signals), checking that communication remains stable (use an EMI filter or shielding if needed to meet this requirement). |

### 7. Bill of Materials (BOM)

*What major components do you need and why? Try to be as specific as possible. Your Hardware & Software Requirements Specifications should inform your component choices.*

*In addition to this written response, copy the Final Project BOM Google Sheet and fill it out with your critical components (think: processors, sensors, actuators). Include the link to your BOM in this section.*

ATMega 328PB

This will be our main microntroller that will control the motors and interface between the sensors.

Ultrasonic sensor

The ultrasonic sensor will be positioned on top of the vehicle to detect the distance to the nearest wall or large obstacle. This way, we can avoid collisions.

Logic level shifter

Since the digital high of the ATMega328pb and the raspberry pi are different (5 V vs 3.3 V), and we want to communicate through serial, we require a logic level shifter between them. We have chosen a bidirectional logic shifter from adafruit since it will ensure we are able to communicate using I2C.

Infrarred sensor

We have chosen an analog infrarred sensor, since it is more precise than the ultrasonic sensor and it has a longer range than digital infrarred sensors. It will be placed at floor level, facing forward, to detect the "trash" that the vehicle should collect.

Raspberry Pi

The raspberry pi will be used for image processing. We will be using this board instead of the ATMega because image processing is very computationally intensive, and we believe that using the ATMega would greatly limit the real-time

Camera

We will use the Raspberry Pi cam v3 because it can interface with the raspberry pi directly. This way, the raspberry pi can do the necessary image processing directly.

Car chassis

To build this, we will design a base board, laser cut it with acrylic and attach the motors on the bottom and the main components. This is

Wheels

There will be two front wheels, each attached to a motor.

An omnidirectlional back wheel bearing will be used so the

Motor

We have chosen ____ motor because ____. We will have two motors, one for each of the front wheels that we will be able to actuate independently to steer the car.

Motor driver

The chosen

### 8. Final Demo Goals

*How will you demonstrate your device on demo day? Will it be strapped to a person, mounted on a bicycle, require outdoor space? Think of any physical, temporal, and other constraints that could affect your planning.*

We will require enough floor space to demonstrate the vehicle moving. We will also require small objects that the system can detect and dispose of.

### 9. Sprint Planning

*You've got limited time to get this project done! How will you plan your sprint milestones? How will you distribute the work within your team? Review the schedule in the final project manual for exact dates.*

| Milestone  | Functionality Achieved                                                                                              | Distribution of Work                  |
| ---------- | ------------------------------------------------------------------------------------------------------------------- | ------------------------------------- |
| Sprint #1  | We want to finish the lasercutting, attachment, and wiring of the base robot and its connected hardware components. | We will work on this together equally |
| Sprint #2  | We want the robot to be able to move and sense and communicate distances.                                           | We will work on this together equally |
| MVP Demo   | We want to be able to perform image processing for trash detection, and correspondingly move the robot to           | We will work on this together equally |
| Final Demo |                                                                                                                     | We will work on this together equally |

**This is the end of the Project Proposal section. The remaining sections will be filled out based on the milestone schedule.**

## Sprint Review #1

### Last week's progress

### Current state of project

### Next week's plan

## Sprint Review #2

### Last week's progress

### Current state of project

### Next week's plan

## MVP Demo

1. Show a system block diagram & explain the hardware implementation.
2. Explain your firmware implementation, including application logic and critical drivers you've written.
3. Demo your device.
4. Have you achieved some or all of your Software Requirements Specification (SRS)?

   1. Show how you collected data and the outcomes.
5. Have you achieved some or all of your Hardware Requirements Specification (HRS)?

   1. Show how you collected data and the outcomes.
6. Show off the remaining elements that will make your project whole: mechanical casework, supporting graphical user interface (GUI), web portal, etc.
7. What is the riskiest part remaining of your project?

   1. How do you plan to de-risk this?
8. What questions or help do you need from the teaching team?

## Final Project Report

Don't forget to make the GitHub pages public website!
If you’ve never made a GitHub pages website before, you can follow this webpage (though, substitute your final project repository for the GitHub username one in the quickstart guide):  [https://docs.github.com/en/pages/quickstart](https://docs.github.com/en/pages/quickstart)

### 1. Video

[Insert final project video here]

* The video must demonstrate your key functionality.
* The video must be 5 minutes or less.
* Ensure your video link is accessible to the teaching team. Unlisted YouTube videos or Google Drive uploads with SEAS account access work well.
* Points will be removed if the audio quality is poor - say, if you filmed your video in a noisy electrical engineering lab.

### 2. Images

[Insert final project images here]

*Include photos of your device from a few angles. If you have a casework, show both the exterior and interior (where the good EE bits are!).*

### 3. Results

*What were your results? Namely, what was the final solution/design to your problem?*

#### 3.1 Software Requirements Specification (SRS) Results

*Based on your quantified system performance, comment on how you achieved or fell short of your expected requirements.*

*Did your requirements change? If so, why? Failing to meet a requirement is acceptable; understanding the reason why is critical!*

*Validate at least two requirements, showing how you tested and your proof of work (videos, images, logic analyzer/oscilloscope captures, etc.).*

| ID     | Description                                                                                               | Validation Outcome                                                                          |
| ------ | --------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------- |
| SRS-01 | The IMU 3-axis acceleration will be measured with 16-bit depth every 100 milliseconds +/-10 milliseconds. | Confirmed, logged output from the MCU is saved to "validation" folder in GitHub repository. |

#### 3.2 Hardware Requirements Specification (HRS) Results

*Based on your quantified system performance, comment on how you achieved or fell short of your expected requirements.*

*Did your requirements change? If so, why? Failing to meet a requirement is acceptable; understanding the reason why is critical!*

*Validate at least two requirements, showing how you tested and your proof of work (videos, images, logic analyzer/oscilloscope captures, etc.).*

| ID     | Description                                                                                                                        | Validation Outcome                                                                                                      |
| ------ | ---------------------------------------------------------------------------------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------- |
| HRS-01 | A distance sensor shall be used for obstacle detection. The sensor shall detect obstacles at a maximum distance of at least 10 cm. | Confirmed, sensed obstacles up to 15cm. Video in "validation" folder, shows tape measure and logged output to terminal. |
|        |                                                                                                                                    |                                                                                                                         |

### 4. Conclusion

Reflect on your project. Some questions to address:

* What did you learn from it?
* What went well?
* What accomplishments are you proud of?
* What did you learn/gain from this experience?
* Did you have to change your approach?
* What could have been done differently?
* Did you encounter obstacles that you didn’t anticipate?
* What could be a next step for this project?

## References

Fill in your references here as you work on your final project. Describe any libraries used here.
for wal
 C
tput
nsor detection. 
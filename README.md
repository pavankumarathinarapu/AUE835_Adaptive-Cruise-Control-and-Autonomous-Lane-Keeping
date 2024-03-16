# AUE835_Adaptive-Cruise-Control-and-Autonomous-Lane-Keeping
To design and implement the micro-controller Arduino UNO on Ultrasonic Sensor HC-SR04 run down the hill as fast as possible while achieving the following function. 
• To Stop the RC Vehicle 30cm away from the obstacle in front • The vehicle needs to maintain 30cm always when in motion

AuE835 Pavan Kumar Athinarapu Project Report
Adaptive Cruise Control 
1. Adaptive Cruise Control
1.1 Problem Statement
To design and implement the micro-controller Arduino UNO on Ultrasonic Sensor HC-SR04 run down the hill as fast as possible while achieving the following function.
• To Stop the RC Vehicle 30cm away from the obstacle in front
• The vehicle needs to maintain 30cm always when in motion
1.2 Technical Approach
We have used an Ultrasonic sensor to calculate the distance from the obstacles. A non-model-based full-state feedback controller is used to achieve Adaptive cruise control. Proportional Based Control is used to control the throttle
• Limiting time data from the sensor

AuE835 Pavan Kumar Athinarapu Project Report
Autonomous lane keeping

1) Logical limits for possible sensor output
• Throttle Error Calculation
• High-Resolution Throttle Control (1500 neutral)
• Proportional Controller Evolution
3) Simple approach first i.e., error*Kp
4) Calibrated dead zones of no throttle response
• Separate Proportional Gain for forward and reverse throttle control.
• Limited the forward throttle value to 1572 to limit speed.
AuE835 Pavan Kumar Athinarapu Project Report
2
PID Controller:
• The PID controller acts as the feedback mechanism used in a control system. The PID control stands for Proportional-Integrative-Derivative control. The PID controller is the non-modal-based controller.
• We have used separate proportional values for front and reverse throttle.
1.3 Hardware and Software Implementation
a. Embedded System and Arduino Uno R3:
• An embedded system is a computer system with a dedicated function within a larger mechanical or electrical system, often with real-time computing constraints.
• Arduino is an open-source embedded system based on easy-to-use hardware and software (https://www.Arduino.cc).
• The Arduino UNO is a microcontroller board that consists of 14 digital input/output pins which are used for a variety of electronics projects.
AuE835 Pavan Kumar Athinarapu Project Report
3
b. Bread Board:
A breadboard, solderless breadboard, or protoboard is a construction base used to build semi-permanent prototypes of electronic circuits.

c. Ultrasonic Sensor:
This is a contactless measurement sensor. It can measure distance up to 2cm – 400cm. Its accuracy is about three millimeters which is very efficient in water level and depth measurement applications.

d. Jumper Wires (M-M & M-F):
Jumper wires are simply wires that have connector pins at each end, allowing them to be used to connect two points to each other without soldering. Jumper wires are typically used with breadboards and other prototyping tools to make it easy to change a circuit as needed. Fairly simple. In fact, it doesn’t get much more basic than jumper wires.
e. Red, Green LEDs, and Buzzer
f. RC Car

Software Implementation:
We have used Proportional Control for the project

5
1.4 Experimental Results
2. Autonomous Lane Keeping
2.1 Problem Statement
To design and implement the micro-controller Arduino UNO on Ultrasonic Sensor HC-SR04 run down the hill by performing the autonomous lane-keeping function as fast as possible.
2.2 Technical Approach
The PID controller acts as the feedback mechanism used in a control system. The PID control stands for Proportional-Integrative-Derivative control. The PID controller is the non-modal-based controller.
• Centre Error Calculation
• Proportional Controller Evolution
1) Simple approach first i.e., error*Kp
2) Error limit to control maximum steering input
3) 2-Stage Proportional Gain for steering control
4) Low proportional gain for steering within a small error
5) High proportional gain for steering during large error
• Integration with Adaptive Cruise Control
• Skipped throttle correction every fourth loop calculation
2.3 Hardware and Software Implementation+
a. Embedded System and Arduino Uno R3:
b. Bread Board:
c. Ultrasonic Sensor:
6
d. Jumper Wires (M-M & M-F)
e. Red, Green LEDs, and Buzzer
f. RC Car
Software Implementation:
We have used Proportional Control for the project
Figure 11: Steering Control Code
2.4 Experimental Results
7
3. Conclusions and Discussions
3.1 Conclusions
• The RC vehicle is kept along the defined center line and stopped 30 cm away from the object.
Limitations for both approaches:
• The data is observed from the ultrasonic sensor.
• Speed of the motor depends on the state of charge of the battery.
3.2 Discussions
1) For Throttle initially we used base throttle code, but we were not getting the desired output. So, the reason we have taken high-resolution throttle code.
2) Limited time data from sensor logical time limits for possible sensor output
3) We used proportional controller evaluation for error calculation
4) Simple approach first i.e., error*kp
5) Calibrated dead zones for no throttle response.
6) Used separate proportional gains for forward and reverse throttle control.
7) Since it is proportional gain throttle was reaching its maximum speed based on distance, so irrespective of gain we have limited the throttle input to 1572
8) Sensor placement was very crucial for steering control. We put the sensor for steering control right above the front wheels to get the best response for a change in direction.
9) The time data was limited to logical limits of the distances we could measure inside the track.
10) interpolation method was used to calculate distance from time data.
11) The center error calculation was done considering the sensor position.
12) Fun part- the first simple approach was used but the results were neither predictable nor repeatable.
13) limiting the error to control maximum steering output in either direction worked well but we still didn't clear the checkpoints.
14) The 2-stage proportional gain was the final iteration which worked perfectly with our setup.
15) Integration with the Adaptive Cruise control was something we achieved successfully by skipping the throttle correction every fourth loop correction.

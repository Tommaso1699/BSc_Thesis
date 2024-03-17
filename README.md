# Line_follower_robot_with_mobile_application 
<h2>About program:</h2>
The goal of my Engineering Thesis was to build the Line follower robot and implement code for it, which would allow it to drive on the track. In this repository
you can find the code for Line Follower robot written for Arduino and also sketch of mobile application, which is used to send PID controller settings via Bluetooth module 
placed on robot and implement these settings inside robot's source code.
<h2>How to run it:</h2>
To run the code for Line Follower you need to install Arduino IDE. Later you need to connect computer with Arduino and upload the code to it.
To run mobile application you need to upload it for a mobile phone containing Android software.
<h2>Code for Line Follower in shortcut:</h2>

1. Attach libraries,
2. Declare global variables,
3. Define aliases,
4. Set the pins in the correct states inside void setup() function,
5. Set speed of data transmission,
6. Calibrate the optocouplers,
7. Map the values ​​read by optocouplers,
8. Call the PID_control() function,
9. Read the current position of the robot based on the interpretation obtained from the optocouplers,
10. Calculate PID controller settings,
11. Calculate speed correction by using PID controller,
12. Send values to robot's engines,
13. Call the void drive(int, int) function, which allows robot to drive.
<h2>Code of application in shortcut:</h2>

1. Search for an available Bluetooth module in the phone settings,
2. Connect to the Bluetooth module placed on Line follower robot,
3. Open the mobile application,
4. Select the Line follower's Bluetooth module available on the list inside Bluetooth label,
5. Enter PID controller setting in the text fields,
6. Send the values by clicking buttons as follows: Send K, Send Ki, Send Kd,
7. If necessary, change the values inside text boxes and then send them again to Arduino by clicking buttons as follows: Send K, Send Ki, Send Kd.



# Person-Following-Car
This project uses a hobby grade RC car that is controlled by my laptop using an Arduino. Matlab code on my laptop finds a person using face detection, checkerboard following or color following. The computer then tells the arduino throttle commands and the arduino maps that to PWM commands for the ESC and Servo on the RC car.

Read the README.pdf for a more in-depth explanation


Use the matlab scripts that start with Follow to do the person tracking. Pick following type you want (face, checkerboard, color)

Use the serialcarcontrol code for the arduino. 


The navigation code is another matlab code that just demonstrates a navigation scheme I came up with.
The navigation scheme does not know where objects are before it reaches them. It only reacts to objects
it finds. This is similar to a person who knows approximately where they want to go, but doesn’t have a map
to find the best path. 
You can change the start and goal locations to see different paths the robot takes. 

Link to video of code running:
https://www.youtube.com/watch?time_continue=1&v=jminU7njedA

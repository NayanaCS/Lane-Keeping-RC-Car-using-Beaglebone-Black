# Lane-Keeping-RC-Car-using-Beaglebone-Black

# code and details uploaded to hackster.io
https://www.hackster.io/wall-ee/lane-keeping-rc-car-using-beaglebone-black-0da1f3

# Story 
We made an autonomous, lane-following RC car by modifying existing Python code for use with the Beaglebone Black. We used two of the BeagleBone's PWM pins to adjust the throttle and steering of the RC car. For lane tracking, we used OpenCV to detect lane edges and adjust the trajectory of our car appropriately.

In order to utilize a wide range of steering values, we used a PD controller to adjust our car's steering angle by changing the PWM value according to how much its trajectory deviated from the path. We resized our frame to 60x40 pixels to improve our car's reaction times. We used a small proportional gain to minimize overcorrections in the steering.

On top of this, we also added some exciting new features! Every third frame, we used OpenCV's masking tools to only detect red HSV values in our frames. We then divided the number of red pixels by the total number of pixels to see the percentage of red in the frame. If enough red is detected, our car will recognize it as a stop sign or stop light and come to a stop, restarting after five seconds. We have handled the two stop signs and Red traffic light in the project.

Inspiration: User raja_961, “Autonomous Lane-Keeping Car Using RaspberryPi and OpenCV”. Instructables. URL:https://www.instructables.com/Autonomous-Lane-Keeping-Car-Using-Raspberry-Pi-and/

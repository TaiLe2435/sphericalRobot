# sphericalRobot

Code for tracking a spherical robot using an IMU and vision system.

The Filter files are the IMU code, make sure to download the file and have all those files in the same directory when running the code. As mentioned before, yaw data is the only thing that needs work. 

Tracking.py is the software for the iteractive vision system. Change the upper and lower bounds of the mask to decide on the color that is going to be tracked. 

passDat.py and readDatafromPython.ino is basic code I created to understand Python and Arduino serial communication. Basically Python will send a string with an added "\r" to indicate the end of the string. The arduino will listen for these bytes and when it gets it, will the bytes to strings (or ints if you wanted) and turn on or off the onboard LED. For our purpose though, we will need to maybe look into multithreading or interrupts, so that the IMU data gets calculated at the same times as the Python data is getting received. Or, if it turns out that the ESP32 can do everything fast enough without it we can deal with it later. 

021926 - This package is the initial test to send distance data like so:

ultrasonic sensors --[piduino_test_01.ino]--> arduino --[usb serial]--> raspberry pi --[ros2]--> publisher.py --[ros2]--> subscriber.py

As of this build, all .py files were writen by Chad Gipiti and still need to be tested. piduino_test_01.ino is a slightly-altered version of the (working) PPP arduino sketch,
modified to output a plain float value for each sensor to usb serial in a (hopefully) reliable way.
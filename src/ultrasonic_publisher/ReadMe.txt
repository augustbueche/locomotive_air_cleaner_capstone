021926 - This package is the initial test to send distance data like so:

ultrasonic sensors --[piduino_test_01.ino]--> arduino --[usb serial]--> raspberry pi --[ros2]--> publisher.py --[ros2]--> subscriber.py

piduino_test_01.ino is a slightly-altered version of the (working) PPP arduino sketch,
modified to output a plain float value for each sensor to usb serial in a (hopefully) reliable way.

As of this build, everything works, but it's a bit sluggish to update the subscriber with the latest distances. hopefully that's just a serial print thing and an actual response would be faster. I guess we'll find out...

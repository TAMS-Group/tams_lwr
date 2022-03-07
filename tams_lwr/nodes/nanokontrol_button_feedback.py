#!/usr/bin/env python
#
# basic "button feedback" via the LEDs on the
# KORG nanokontrol MIDI controller. When the uses presses
# a button, the corresponding LEDs is switched on.
#
# Author: Norman Hendrich, hendrich@informatik.uni-hamburg.de
#
# Note that the device defaults to "internal" LED control,
# where the button LEDs are active while the button is pressed,
# or depending on the selected workstation mode (Cubase, etc.).
#
# To enable "external" LED control, install and run the
# Korg Kontrol Editor program (e.g., using Wine). Then:
# (Control drop-down list) -> Common -> LED Mode -> External
# Write Scene.
#
import rospy
from sensor_msgs.msg import Joy


def buttonFeedbackCallback(data):
    ledMsg = Joy()
    ledMsg.header.stamp = rospy.Time.now()
    ledMsg.buttons = data.buttons
    ledPublisher.publish(ledMsg)


def main():
    rospy.Subscriber("nanokontrol", Joy, buttonFeedbackCallback)
    while not rospy.is_shutdown():
        rospy.spin()
        r.sleep()


if __name__ == "__main__":
    rospy.init_node("nanokontrol_button_feedback")
    # Overall loop rate, anything from 1..10 Hz perhaps
    rate = int(rospy.get_param("~rate", 10))
    r = rospy.Rate(rate)
    ledPublisher = rospy.Publisher("nanokontrol_leds", Joy)
    main()

#!/usr/bin/env python
#
# simple demonstration of "external" LED control on the
# KORG nanokontrol MIDI controller.
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

import roslib
import rospy
from sensor_msgs.msg import JoyFeedback
from nano_common import CC_CYCLE, CC_SOLO_0

roslib.load_manifest("tams_lwr")


def main():
    rospy.init_node("nanokontrol_led_demo")

    # Overall loop rate, anything from 1..10 Hz perhaps
    rate = int(rospy.get_param("~rate", 10))
    r = rospy.Rate(rate)

    led_publisher = rospy.Publisher("nanokontrol_feedback", JoyFeedback)
    iteration = 0

    while not rospy.is_shutdown():
        iteration = iteration + 1
        # blink the 'cycle' LED
        led_msg = JoyFeedback()
        led_msg.type = JoyFeedback.TYPE_LED
        led_msg.id = CC_CYCLE  # node wants the actual MIDI code, not this: button_mapping[ CC_CYCLE ]
        led_msg.intensity = 0.5 * (iteration & 0x1)
        led_publisher.publish(led_msg)

        # binary count sequence on the channel 'S' buttons
        for i in range(7):
            led_msg.type = JoyFeedback.TYPE_LED
            led_msg.intensity = 0.5 * min(1, (iteration & (1 << i)))
            led_msg.id = CC_SOLO_0 + i
            led_publisher.publish(led_msg)
        r.sleep()


if __name__ == "__main__":
    main()
